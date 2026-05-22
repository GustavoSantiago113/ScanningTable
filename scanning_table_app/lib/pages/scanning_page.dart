import 'dart:async';
import 'dart:convert';
import 'dart:io';

import 'package:camera/camera.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:http/http.dart' as http;
import 'package:path_provider/path_provider.dart';
import 'package:permission_handler/permission_handler.dart';
import 'package:wakelock_plus/wakelock_plus.dart';

import '../main.dart' show kPrimary, kDark, kEspBaseUrl;

const Duration kCaptureDelay = Duration(milliseconds: 1000);

class ScanningPage extends StatefulWidget {
  final List<CameraDescription> cameras;
  const ScanningPage({super.key, required this.cameras});

  @override
  State<ScanningPage> createState() => _ScanningPageState();
}

class _ScanningPageState extends State<ScanningPage> {
  // --- Camera ---
  CameraController? _camera;
  bool _cameraReady = false;

  // --- SAF ---
  final MethodChannel _safChannel = const MethodChannel('com.gustavo.saf');
  String? _safTreeUri;

  // --- Sequence state ---
  bool _sequenceStarted = false;
  bool _capturing = false;
  int _lastProcessedStop = 0;
  Timer? _pollTimer;

  @override
  void initState() {
    super.initState();
    WakelockPlus.enable();
    _initCamera();
  }

  @override
  void dispose() {
    WakelockPlus.disable();
    _pollTimer?.cancel();
    _camera?.dispose();
    super.dispose();
  }

  // ---------------------------------------------------------------------------
  // Camera
  // ---------------------------------------------------------------------------

  Future<void> _initCamera() async {
    if (widget.cameras.isEmpty) return;
    final camPerm = await Permission.camera.request();
    if (!camPerm.isGranted) return;

    // Prefer the first back-facing camera.
    final desc = widget.cameras.firstWhere(
      (c) => c.lensDirection == CameraLensDirection.back,
      orElse: () => widget.cameras.first,
    );

    final controller = CameraController(
      desc,
      ResolutionPreset.max,
      enableAudio: false,
      imageFormatGroup: ImageFormatGroup.jpeg,
    );
    await controller.initialize();

    // Lock focus and exposure for repeatable shots.
    // Note: shutter speed (1/4 s) and ISO (200) require platform-specific
    // Camera2 API and cannot be set directly via the Flutter camera plugin.
    // Aperture (f/2.4) is fixed in hardware on most phone cameras.
    try { await controller.setFocusMode(FocusMode.locked); } catch (_) {}
    try { await controller.setExposureMode(ExposureMode.locked); } catch (_) {}

    if (!mounted) return;
    setState(() {
      _camera = controller;
      _cameraReady = true;
    });
    // Apply Camera2 manual settings: 1/4 s shutter, ISO 200, f/2.4.
    try {
      await _safChannel.invokeMethod<bool>('applyCamera2Settings');
    } catch (e) {
      debugPrint('Camera2 settings not applied: $e');
    }
  }

  // ---------------------------------------------------------------------------
  // Folder picker
  // ---------------------------------------------------------------------------

  Future<void> _pickFolder() async {
    try {
      final uri = await _safChannel.invokeMethod<String>('pickDirectory');
      if (uri != null && mounted) {
        setState(() => _safTreeUri = uri);
        ScaffoldMessenger.of(context).showSnackBar(
          const SnackBar(content: Text('Folder selected')),
        );
      }
    } catch (e) {
      debugPrint('Pick folder error: $e');
    }
  }

  // ---------------------------------------------------------------------------
  // Photo saving
  // ---------------------------------------------------------------------------

  String _ts(DateTime dt) {
    String two(int n) => n.toString().padLeft(2, '0');
    return '${dt.year}${two(dt.month)}${two(dt.day)}'
        '_${two(dt.hour)}${two(dt.minute)}${two(dt.second)}';
  }

  Future<Directory> _photosDir() async {
    try {
      if (Platform.isAndroid) {
        PermissionStatus storageStatus;
        if (await Permission.photos.isGranted ||
            await Permission.photos.request().isGranted) {
          storageStatus = PermissionStatus.granted;
        } else {
          storageStatus = await Permission.storage.request();
        }
        if (storageStatus.isGranted) {
          final downloads = Directory('/storage/emulated/0/Download');
          if (!await downloads.exists()) await downloads.create(recursive: true);
          return downloads;
        }
      }
    } catch (_) {}
    final appDir = await getApplicationDocumentsDirectory();
    final dir = Directory('${appDir.path}${Platform.pathSeparator}ScanningTable');
    if (!await dir.exists()) await dir.create(recursive: true);
    return dir;
  }

  Future<void> _handleStopEvent(int stopIndex) async {
    if (!_cameraReady || _capturing) return;
    if (mounted) setState(() => _capturing = true);
    try {
      final name =
          'stop_${stopIndex.toString().padLeft(2, '0')}_${_ts(DateTime.now())}.jpg';
      final XFile shot = await _camera!.takePicture();
      final bytes = await File(shot.path).readAsBytes();

      if (_safTreeUri != null) {
        try {
          final ok = await _safChannel.invokeMethod<bool>(
            'saveFileToDirectory',
            {
              'treeUri': _safTreeUri,
              'filename': name,
              'base64': base64Encode(bytes),
            },
          );
          if (ok != true) {
            final dir = await _photosDir();
            await File(shot.path)
                .copy('${dir.path}${Platform.pathSeparator}$name');
          }
        } catch (_) {
          final dir = await _photosDir();
          await File(shot.path)
              .copy('${dir.path}${Platform.pathSeparator}$name');
        }
      } else {
        final dir = await _photosDir();
        await File(shot.path)
            .copy('${dir.path}${Platform.pathSeparator}$name');
      }

      await Future.delayed(kCaptureDelay);
      await _httpContinue();
    } catch (e) {
      debugPrint('Capture error: $e');
    } finally {
      if (mounted) setState(() => _capturing = false);
    }
  }

  // ---------------------------------------------------------------------------
  // HTTP
  // ---------------------------------------------------------------------------

  Uri _uri(String path, [Map<String, String>? q]) =>
      Uri.parse('$kEspBaseUrl$path').replace(queryParameters: q);

  Future<void> _httpStart() async {
    setState(() {
      _sequenceStarted = true;
      _lastProcessedStop = 0;
    });
    try {
      final resp = await http
          .post(_uri('/start', {'turns': '1'}))
          .timeout(const Duration(seconds: 5));
      if (resp.statusCode == 200) {
        _startPolling();
      } else {
        if (mounted) setState(() => _sequenceStarted = false);
      }
    } catch (_) {
      if (mounted) setState(() => _sequenceStarted = false);
    }
  }

  Future<void> _httpStop() async {
    _pollTimer?.cancel();
    try {
      await http.post(_uri('/stop')).timeout(const Duration(seconds: 5));
    } catch (_) {}
    if (mounted) {
      setState(() {
        _sequenceStarted = false;
        _capturing = false;
      });
    }
  }

  Future<void> _httpContinue() async {
    try {
      await http.post(_uri('/continue')).timeout(const Duration(seconds: 5));
    } catch (_) {}
  }

  void _startPolling() {
    _pollTimer?.cancel();
    _pollTimer =
        Timer.periodic(const Duration(milliseconds: 400), (_) async {
      try {
        final resp = await http
            .get(_uri('/status'))
            .timeout(const Duration(seconds: 5));
        if (resp.statusCode != 200) return;
        final data = jsonDecode(resp.body) as Map<String, dynamic>;
        final running = data['running'] == true;
        final current = (data['currentStop'] ?? 0) as int;
        final total = (data['totalStops'] ?? 0) as int;

        if (mounted) setState(() => _sequenceStarted = running);

        if (current > _lastProcessedStop) {
          _lastProcessedStop = current;
          await _handleStopEvent(current);
        }
        if (!running && total > 0 && current >= total) {
          _pollTimer?.cancel();
          if (mounted) setState(() => _sequenceStarted = false);
        }
      } catch (_) {}
    });
  }

  // ---------------------------------------------------------------------------
  // UI
  // ---------------------------------------------------------------------------

  @override
  Widget build(BuildContext context) {
    final canScan = _cameraReady &&
        _safTreeUri != null &&
        !_sequenceStarted &&
        !_capturing;
    final canStop = _sequenceStarted || _capturing;

    return Scaffold(
      backgroundColor: Colors.white,
      appBar: AppBar(
        title: const Text('Scanning Table'),
        backgroundColor: kPrimary,
        foregroundColor: Colors.white,
        elevation: 0,
        centerTitle: true,
      ),
      body: SafeArea(
        child: Column(
          children: [
            Expanded(child: _buildPreview()),
            _buildControls(canScan: canScan, canStop: canStop),
          ],
        ),
      ),
    );
  }

  Widget _buildPreview() {
    if (!_cameraReady || _camera == null) {
      return const Center(child: CircularProgressIndicator());
    }
    return ColoredBox(
      color: Colors.black,
      child: Center(
        child: AspectRatio(
          aspectRatio: 3 / 4,
          child: CameraPreview(_camera!),
        ),
      ),
    );
  }

  Widget _buildControls({required bool canScan, required bool canStop}) {
    return DecoratedBox(
      decoration: const BoxDecoration(
        color: Colors.white,
        boxShadow: [BoxShadow(color: Colors.black12, blurRadius: 8, offset: Offset(0, -2))],
      ),
      child: Padding(
        padding: const EdgeInsets.fromLTRB(20, 16, 20, 20),
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            // Folder picker
            SizedBox(
              width: double.infinity,
              child: OutlinedButton.icon(
                onPressed: _pickFolder,
                style: OutlinedButton.styleFrom(
                  foregroundColor: kDark,
                  side: BorderSide(
                    color: _safTreeUri != null ? kPrimary : Colors.black26,
                  ),
                  padding: const EdgeInsets.symmetric(vertical: 14),
                  shape: RoundedRectangleBorder(
                      borderRadius: BorderRadius.circular(12)),
                ),
                icon: Icon(
                  _safTreeUri != null
                      ? Icons.folder_rounded
                      : Icons.folder_open_rounded,
                  color: _safTreeUri != null ? kPrimary : Colors.black45,
                ),
                label: Text(
                  _safTreeUri != null
                      ? 'Save folder selected ✓'
                      : 'Select save folder',
                  style: TextStyle(
                    color: _safTreeUri != null ? kPrimary : Colors.black45,
                  ),
                ),
              ),
            ),
            if (_safTreeUri == null)
              const Padding(
                padding: EdgeInsets.only(top: 6),
                child: Text(
                  'Select a folder before scanning.',
                  style: TextStyle(color: Colors.redAccent, fontSize: 12),
                ),
              ),
            const SizedBox(height: 14),

            // Scan / Stop buttons
            Row(
              children: [
                Expanded(
                  child: ElevatedButton.icon(
                    onPressed: canScan ? _httpStart : null,
                    style: ElevatedButton.styleFrom(
                      backgroundColor: kPrimary,
                      foregroundColor: Colors.white,
                      disabledBackgroundColor: kPrimary.withOpacity(0.3),
                      padding: const EdgeInsets.symmetric(vertical: 16),
                      shape: RoundedRectangleBorder(
                          borderRadius: BorderRadius.circular(14)),
                    ),
                    icon: _capturing
                        ? const SizedBox(
                            width: 18,
                            height: 18,
                            child: CircularProgressIndicator(
                                strokeWidth: 2, color: Colors.white),
                          )
                        : const Icon(Icons.document_scanner_rounded),
                    label: Text(
                      _capturing ? 'Capturing…' : 'Scan',
                      style: const TextStyle(
                          fontSize: 16, fontWeight: FontWeight.w700),
                    ),
                  ),
                ),
                const SizedBox(width: 12),
                Expanded(
                  child: ElevatedButton.icon(
                    onPressed: canStop ? _httpStop : null,
                    style: ElevatedButton.styleFrom(
                      backgroundColor: Colors.redAccent,
                      foregroundColor: Colors.white,
                      disabledBackgroundColor: Colors.redAccent.withOpacity(0.3),
                      padding: const EdgeInsets.symmetric(vertical: 16),
                      shape: RoundedRectangleBorder(
                          borderRadius: BorderRadius.circular(14)),
                    ),
                    icon: const Icon(Icons.stop_rounded),
                    label: const Text(
                      'Stop',
                      style:
                          TextStyle(fontSize: 16, fontWeight: FontWeight.w700),
                    ),
                  ),
                ),
              ],
            ),

            // Progress indicator
            if (_sequenceStarted || _capturing) ...[
              const SizedBox(height: 10),
              Row(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  const SizedBox(
                    width: 12,
                    height: 12,
                    child: CircularProgressIndicator(
                        strokeWidth: 2, color: kPrimary),
                  ),
                  const SizedBox(width: 8),
                  Text(
                    _capturing
                        ? 'Capturing photo…'
                        : 'Scanning — stop $_lastProcessedStop / 36',
                    style: const TextStyle(
                        color: Colors.black54, fontSize: 13),
                  ),
                ],
              ),
            ],
          ],
        ),
      ),
    );
  }
}
