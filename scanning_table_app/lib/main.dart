import 'dart:async';
import 'dart:io';
import 'package:camera/camera.dart';
import 'package:flutter/material.dart';
import 'package:path_provider/path_provider.dart';
import 'package:web_socket_channel/io.dart';
import 'package:web_socket_channel/status.dart' as ws_status;
import 'package:permission_handler/permission_handler.dart';

const kPrimary = Color(0xFF05989E);
const kDark = Color(0xFF3C444B);

// Default number of turns sent to ESP (user only picks stops)
const int kDefaultTurns = 6;

// ESP8266 WS endpoint (SoftAP IP)
const String kEspWsUrl = 'ws://192.168.4.1:81';

Future<void> main() async {
  WidgetsFlutterBinding.ensureInitialized();
  final cameras = await availableCameras();
  runApp(App(cameras: cameras));
}

class App extends StatelessWidget {
  final List<CameraDescription> cameras;
  const App({super.key, required this.cameras});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Stepper Camera',
      debugShowCheckedModeBanner: false,
      theme: ThemeData(
        useMaterial3: true,
        colorScheme: ColorScheme.fromSeed(seedColor: kPrimary, primary: kPrimary, secondary: kDark),
        scaffoldBackgroundColor: Colors.white,
        fontFamily: 'Montserrat',
        textTheme: const TextTheme(
          headlineLarge: TextStyle(fontWeight: FontWeight.w700, color: Colors.white),
          headlineSmall: TextStyle(fontWeight: FontWeight.w700, color: Colors.white),
          titleMedium: TextStyle(fontWeight: FontWeight.w600),
          bodyMedium: TextStyle(fontWeight: FontWeight.w500),
        ),
      ),
      home: HomeScreen(cameras: cameras),
    );
  }
}

class HomeScreen extends StatefulWidget {
  final List<CameraDescription> cameras;
  const HomeScreen({super.key, required this.cameras});

  @override
  State<HomeScreen> createState() => _HomeScreenState();
}

class _HomeScreenState extends State<HomeScreen> {
  IOWebSocketChannel? _channel;
  StreamSubscription? _wsSub;
  bool _connecting = false;
  String _wsStatus = 'Disconnected';

  double _zoom = 1.0;
  double _minZoom = 1.0;
  double _maxZoom = 1.0;

  CameraController? _camera;
  bool _cameraReady = false;
  bool _capturing = false;

  int _selectedCameraIndex = 0;

  final _stopsCtrl = TextEditingController(text: '12');
  final List<String> _log = [];
  File? _lastPhoto;

  @override
  void initState() {
    super.initState();
    _initCameraForIndex(_selectedCameraIndex).then((_) => _connectWs());
  }

  @override
  void dispose() {
    _wsSub?.cancel();
    _channel?.sink.close(ws_status.normalClosure);
    _camera?.dispose();
    _stopsCtrl.dispose();
    super.dispose();
  }

  Future<void> _initCameraForIndex(int index) async {
    try {
      if (widget.cameras.isEmpty) {
        _addLog('No cameras found.');
        return;
      }
      if (index < 0 || index >= widget.cameras.length) index = 0;

      // Request permission
      final camPerm = await Permission.camera.request();
      if (!camPerm.isGranted) {
        _addLog('Camera permission not granted.');
        return;
      }

      final desc = widget.cameras[index];

      // dispose previous controller if any
      if (_camera != null) {
        try {
          await _camera!.dispose();
        } catch (_) {}
        _camera = null;
        _cameraReady = false;
      }

      final controller = CameraController(
        desc,
        ResolutionPreset.medium,
        enableAudio: false,
        imageFormatGroup: ImageFormatGroup.jpeg,
      );

      await controller.initialize();

      // get zoom bounds (some devices support <1.0 for wide)
      _minZoom = await controller.getMinZoomLevel();
      _maxZoom = await controller.getMaxZoomLevel();

      // clamp current zoom into bounds
      final initialZoom = (_minZoom <= 1.0 && 1.0 <= _maxZoom) ? 1.0 : _minZoom;
      await controller.setZoomLevel(initialZoom);

      setState(() {
        _camera = controller;
        _cameraReady = true;
        _zoom = initialZoom;
        _selectedCameraIndex = index;
      });
      _addLog('Camera initialized (${desc.name}, lens: ${desc.lensDirection}). minZoom: ${_minZoom.toStringAsFixed(2)}, maxZoom: ${_maxZoom.toStringAsFixed(2)}');
    } catch (e) {
      _addLog('Camera init error: $e');
      setState(() {
        _cameraReady = false;
      });
    }
  }

  Future<void> _switchCamera(int index) async {
    if (index == _selectedCameraIndex) return;
    _addLog('Switching camera...');
    await _initCameraForIndex(index);
  }

  Future<void> _setZoom(double zoom) async {
    if (_camera != null && _cameraReady) {
      final clamped = zoom.clamp(_minZoom, _maxZoom);
      try {
        await _camera!.setZoomLevel(clamped);
        setState(() => _zoom = clamped);
      } catch (e) {
        _addLog('Zoom error: $e');
      }
    }
  }

  Future<void> _connectWs() async {
    if (_connecting) return;
    setState(() {
      _connecting = true;
      _wsStatus = 'Connecting...';
    });
    try {
      final channel = IOWebSocketChannel.connect(
        Uri.parse(kEspWsUrl),
        pingInterval: const Duration(seconds: 10),
      );
      _wsSub = channel.stream.listen(
        (msg) => _onWsMessage(msg.toString()),
        onDone: () {
          _addLog('WS closed.');
          setState(() => _wsStatus = 'Disconnected');
        },
        onError: (err) {
          _addLog('WS error: $err');
          setState(() => _wsStatus = 'Error');
        },
        cancelOnError: false,
      );
      setState(() {
        _channel = channel;
        _wsStatus = 'Connected';
      });
      _addLog('Connected to $kEspWsUrl');
    } catch (e) {
      _addLog('WS connect failed: $e');
      setState(() => _wsStatus = 'Disconnected');
    } finally {
      setState(() => _connecting = false);
    }
  }

  void _onWsMessage(String msg) {
    _addLog('ESP: $msg');
    // Expected messages: "STOP x/y", "STARTED ...", "DONE", "STOPPED"
    if (msg.startsWith('STOP ')) {
      _handleStopMessage(msg);
    } else if (msg.startsWith('DONE')) {
      _addLog('Sequence finished by ESP.');
    }
  }

  Future<void> _handleStopMessage(String msg) async {
    if (!_cameraReady || _capturing) return;
    setState(() => _capturing = true);

    try {
      final dir = await _photosDir();
      final stopIndex = _parseStopIndex(msg); // from "STOP 3/12"
      final ts = DateTime.now();
      final name = 'stop_${stopIndex?.toString().padLeft(2, '0') ?? 'x'}_${_ts(ts)}.jpg';
      final savePath = '${dir.path}${Platform.pathSeparator}$name';

      _addLog('Capturing photo...');
      final XFile shot = await _camera!.takePicture();
      await File(shot.path).copy(savePath);

      setState(() => _lastPhoto = File(savePath));
      _addLog('Saved: $name');

      // Notify ESP to continue
      _send('CONTINUE');
      _addLog('Sent CONTINUE');
    } catch (e) {
      _addLog('Capture error: $e');
    } finally {
      setState(() => _capturing = false);
    }
  }

  int? _parseStopIndex(String msg) {
    // "STOP 3/12"
    try {
      final parts = msg.split(' ');
      if (parts.length < 2) return null;
      final frac = parts[1].trim();
      final idx = int.tryParse(frac.split('/').first);
      return idx;
    } catch (_) {
      return null;
    }
  }

  Future<Directory> _photosDir() async {
    final base = await getApplicationDocumentsDirectory();
    final dir = Directory('${base.path}${Platform.pathSeparator}ScanningTable');
    if (!await dir.exists()) {
      await dir.create(recursive: true);
    }
    return dir;
  }

  String _ts(DateTime dt) {
    String two(int n) => n.toString().padLeft(2, '0');
    return '${dt.year}${two(dt.month)}${two(dt.day)}_${two(dt.hour)}${two(dt.minute)}${two(dt.second)}';
    }

  void _addLog(String line) {
    setState(() => _log.insert(0, '${DateTime.now().toIso8601String().substring(11, 19)}  $line'));
  }

  void _send(String text) {
    try {
      _channel?.sink.add(text);
    } catch (e) {
      _addLog('Send error: $e');
    }
  }

  void _start() {
    final stops = int.tryParse(_stopsCtrl.text.trim());
    if (stops == null || stops <= 0) {
      _addLog('Invalid stops value.');
      return;
    }
    if (_channel == null) {
      _addLog('Not connected.');
      return;
    }
    _send('START $kDefaultTurns $stops');
    _addLog('Sent START $kDefaultTurns $stops');
  }

  void _stop() {
    _send('STOP');
    _addLog('Sent STOP');
  }

  @override
  Widget build(BuildContext context) {
    return Stack(
      children: [
        // Decorative gradient background
        Container(
          decoration: const BoxDecoration(
            gradient: LinearGradient(
              colors: [kPrimary, kDark],
              begin: Alignment.topLeft,
              end: Alignment.bottomRight,
            ),
          ),
        ),

        // Main content
        Scaffold(
          backgroundColor: Colors.transparent,
          body: SafeArea(
            child: Column(
              children: [
                // Title BEFORE camera
                Padding(
                  padding: const EdgeInsets.symmetric(horizontal: 20.0, vertical: 12),
                  child: Column(
                    children: [
                      Text('Scanning Table', style: Theme.of(context).textTheme.headlineLarge, textAlign: TextAlign.center),
                      const SizedBox(height: 6),
                      Text('ESP8266 + Camera Automation', style: Theme.of(context).textTheme.headlineSmall?.copyWith(color: Colors.white70), textAlign: TextAlign.center),
                    ],
                  ),
                ),

                // Camera selection and big preview
                Padding(
                  padding: const EdgeInsets.symmetric(horizontal: 12.0),
                  child: Card(
                    color: Colors.white.withOpacity(0.06),
                    elevation: 0,
                    shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(14)),
                    child: Column(
                      children: [
                        // camera selector & status
                        Padding(
                          padding: const EdgeInsets.symmetric(horizontal: 12.0, vertical: 8),
                          child: Row(
                            children: [
                              Expanded(
                                child: DropdownButton<int>(
                                  isExpanded: true,
                                  value: _selectedCameraIndex,
                                  dropdownColor: Colors.white,
                                  items: List.generate(widget.cameras.length, (i) {
                                    final c = widget.cameras[i];
                                    final label = '${c.name.isEmpty ? c.lensDirection.name : c.name} (${c.lensDirection.name})';
                                    return DropdownMenuItem(value: i, child: Text(label, style: const TextStyle(color: kDark)));
                                  }),
                                  onChanged: (v) {
                                    if (v != null) _switchCamera(v);
                                  },
                                ),
                              ),
                              const SizedBox(width: 8),
                              Chip(
                                backgroundColor: _cameraReady ? kPrimary.withOpacity(0.12) : Colors.orange.withOpacity(0.12),
                                label: Text(_cameraReady ? 'Camera ready' : 'No camera', style: const TextStyle(color: Colors.white)),
                              )
                            ],
                          ),
                        ),

                        // big preview area (higher height)
                        Container(
                          height: 420, // increased height
                          margin: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
                          decoration: BoxDecoration(
                            borderRadius: BorderRadius.circular(12),
                            color: Colors.black,
                          ),
                          child: ClipRRect(
                            borderRadius: BorderRadius.circular(12),
                            child: _camera != null && _cameraReady
                                ? CameraPreview(_camera!)
                                : Center(child: Text('Camera not available', style: TextStyle(color: Colors.white.withOpacity(0.9)))),
                          ),
                        ),

                        // Zoom slider supporting <1.0 zoom if camera offers it
                        if (_camera != null && _cameraReady)
                          Padding(
                            padding: const EdgeInsets.symmetric(horizontal: 18.0, vertical: 8),
                            child: Row(
                              children: [
                                const Icon(Icons.zoom_out, color: Colors.white),
                                Expanded(
                                  child: Slider(
                                    value: _zoom,
                                    min: _minZoom,
                                    max: _maxZoom,
                                    divisions: 100,
                                    label: _zoom.toStringAsFixed(2),
                                    onChanged: (v) => _setZoom(v),
                                  ),
                                ),
                                const Icon(Icons.zoom_in, color: Colors.white),
                              ],
                            ),
                          ),
                      ],
                    ),
                  ),
                ),

                // Rest of your UI (controls, logs) - scrollable
                Expanded(
                  child: SingleChildScrollView(
                    child: _buildMainContent(context),
                  ),
                ),
              ],
            ),
          ),
        ),
      ],
    );
  }

  Widget _buildMainContent(BuildContext context) {
    return SingleChildScrollView(
      padding: const EdgeInsets.symmetric(horizontal: 20, vertical: 16),
      child: ConstrainedBox(
        constraints: const BoxConstraints(maxWidth: 700),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.stretch,
          children: [
            const SizedBox(height: 12),
            Card(
              color: Colors.white,
              elevation: 6,
              shadowColor: kDark.withOpacity(0.35),
              shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(18)),
              child: Padding(
                padding: const EdgeInsets.all(18.0),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.stretch,
                  children: [
                    Text('Stops', style: Theme.of(context).textTheme.titleMedium),
                    const SizedBox(height: 8),
                    Row(
                      children: [
                        Expanded(
                          child: TextField(
                            controller: _stopsCtrl,
                            keyboardType: TextInputType.number,
                            decoration: InputDecoration(
                              filled: true,
                              fillColor: const Color(0xFFF3F6F8),
                              hintText: 'Enter number of stops',
                              border: OutlineInputBorder(
                                borderRadius: BorderRadius.circular(12),
                                borderSide: BorderSide.none,
                              ),
                              prefixIcon: const Icon(Icons.flag_rounded, color: kDark),
                            ),
                            style: const TextStyle(fontSize: 18),
                          ),
                        ),
                        const SizedBox(width: 12),
                        _pillButton(
                          label: 'Start',
                          icon: Icons.play_arrow_rounded,
                          color: kPrimary,
                          onPressed: (_channel != null && !_capturing) ? _start : null,
                        ),
                      ],
                    ),
                    const SizedBox(height: 12),
                    Row(
                      children: [
                        Expanded(
                          child: _pillButton(
                            label: 'Reconnect',
                            icon: Icons.wifi_tethering_rounded,
                            color: kDark,
                            onPressed: _connecting ? null : _connectWs,
                          ),
                        ),
                        const SizedBox(width: 12),
                        Expanded(
                          child: _pillButton(
                            label: 'Stop',
                            icon: Icons.stop_rounded,
                            color: Colors.redAccent,
                            onPressed: _stop,
                          ),
                        ),
                      ],
                    ),
                  ],
                ),
              ),
            ),

            const SizedBox(height: 18),

            if (_lastPhoto != null)
              _lastPhotoCard(),

            const SizedBox(height: 12),

            _logCard(),
          ],
        ),
      ),
    );
  }

  Widget _pillButton({required String label, required IconData icon, required Color color, VoidCallback? onPressed}) {
    return ElevatedButton.icon(
      onPressed: onPressed,
      style: ElevatedButton.styleFrom(
        backgroundColor: color,
        foregroundColor: Colors.white,
        padding: const EdgeInsets.symmetric(horizontal: 18, vertical: 14),
        shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(14)),
        elevation: 3,
      ),
      icon: Icon(icon),
      label: Text(label, style: const TextStyle(fontWeight: FontWeight.w700)),
    );
  }

  Widget _lastPhotoCard() {
    return Card(
      color: Colors.white,
      elevation: 5,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
      child: Padding(
        padding: const EdgeInsets.all(14.0),
        child: Row(
          children: [
            ClipRRect(
              borderRadius: BorderRadius.circular(12),
              child: Image.file(_lastPhoto!, width: 90, height: 90, fit: BoxFit.cover),
            ),
            const SizedBox(width: 12),
            Expanded(
              child: Column(
                crossAxisAlignment: CrossAxisAlignment.start,
                children: [
                  const Text('Last Capture', style: TextStyle(fontWeight: FontWeight.w700, fontSize: 16)),
                  const SizedBox(height: 4),
                  FutureBuilder<Directory>(
                    future: _photosDir(),
                    builder: (context, snap) {
                      final p = _lastPhoto?.path ?? '';
                      return Text(
                        p.split(Platform.pathSeparator).last,
                        style: const TextStyle(color: kDark),
                        overflow: TextOverflow.ellipsis,
                      );
                    },
                  ),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _logCard() {
    return Card(
      color: Colors.white,
      elevation: 4,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
      child: Container(
        constraints: const BoxConstraints(maxHeight: 220),
        padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 10),
        child: ListView.separated(
          reverse: true,
          itemCount: _log.length,
          separatorBuilder: (_, __) => const Divider(height: 8),
          itemBuilder: (_, i) => Text(_log[i], style: const TextStyle(fontFamily: 'Montserrat')),
        ),
      ),
    );
  }
}