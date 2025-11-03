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
  String _wsStatus = 'Disconnected';
  bool _connecting = false;

  CameraController? _camera;
  bool _cameraReady = false;
  bool _capturing = false;

  final _stopsCtrl = TextEditingController(text: '12');
  final List<String> _log = [];
  File? _lastPhoto;

  @override
  void initState() {
    super.initState();
    _initCamera();
    _connectWs();
  }

  @override
  void dispose() {
    _wsSub?.cancel();
    _channel?.sink.close(ws_status.normalClosure);
    _camera?.dispose();
    _stopsCtrl.dispose();
    super.dispose();
  }

  Future<void> _initCamera() async {
    try {
      if (widget.cameras.isEmpty) {
        _addLog('No cameras found.');
        return;
      }
      // Request permission
      final camPerm = await Permission.camera.request();
      if (!camPerm.isGranted) {
        _addLog('Camera permission not granted.');
        return;
      }
      final back = widget.cameras.firstWhere(
        (c) => c.lensDirection == CameraLensDirection.back,
        orElse: () => widget.cameras.first,
      );
      final controller = CameraController(
        back,
        ResolutionPreset.medium,
        enableAudio: false,
        imageFormatGroup: ImageFormatGroup.jpeg,
      );
      await controller.initialize();
      setState(() {
        _camera = controller;
        _cameraReady = true;
      });
      _addLog('Camera initialized.');
    } catch (e) {
      _addLog('Camera init error: $e');
      setState(() {
        _cameraReady = false;
      });
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
      // Sequence completed
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
    _channel?.sink.add(text);
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
            child: Center(
              child: SingleChildScrollView(
                padding: const EdgeInsets.symmetric(horizontal: 20, vertical: 16),
                child: ConstrainedBox(
                  constraints: const BoxConstraints(maxWidth: 700),
                  child: Column(
                    crossAxisAlignment: CrossAxisAlignment.stretch,
                    children: [
                      const SizedBox(height: 12),
                      Text('Scanning Table', style: Theme.of(context).textTheme.headlineLarge, textAlign: TextAlign.center),
                      const SizedBox(height: 6),
                      Text('ESP8266 + Camera Automation', style: Theme.of(context).textTheme.headlineSmall?.copyWith(color: Colors.white70), textAlign: TextAlign.center),
                      const SizedBox(height: 24),

                      _statusChips(),

                      const SizedBox(height: 18),

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
              ),
            ),
          ),
        ),

        // Keep a hidden camera preview so controller stays active
        if (_camera != null && _cameraReady)
          Positioned.fill(
            child: IgnorePointer(
              child: Opacity(
                opacity: 0.0001,
                child: CameraPreview(_camera!),
              ),
            ),
          ),
      ],
    );
  }

  Widget _statusChips() {
    return Wrap(
      spacing: 10,
      runSpacing: 10,
      alignment: WrapAlignment.center,
      children: [
        _chip(icon: Icons.wifi_rounded, label: _wsStatus, color: _wsStatus == 'Connected' ? kPrimary : Colors.orange),
        _chip(icon: Icons.photo_camera_rounded, label: _cameraReady ? 'Camera ready' : 'Camera not ready', color: _cameraReady ? kPrimary : Colors.orange),
        _chip(icon: Icons.tune_rounded, label: 'Turns: $kDefaultTurns', color: kDark),
      ],
    );
  }

  Widget _chip({required IconData icon, required String label, required Color color}) {
    return Chip(
      avatar: CircleAvatar(backgroundColor: color, child: Icon(icon, size: 16, color: Colors.white)),
      label: Text(label, style: const TextStyle(color: Colors.white)),
      backgroundColor: color.withOpacity(0.2),
      shape: StadiumBorder(side: BorderSide(color: color.withOpacity(0.6))),
      padding: const EdgeInsets.symmetric(horizontal: 8, vertical: 4),
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