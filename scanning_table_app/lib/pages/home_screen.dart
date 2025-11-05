import 'dart:async';
import 'dart:io';
import 'package:camera/camera.dart';
import 'package:flutter/material.dart';
import 'package:path_provider/path_provider.dart';
import 'package:web_socket_channel/io.dart';
import 'package:web_socket_channel/status.dart' as ws_status;
import 'package:permission_handler/permission_handler.dart';
import '../components/components.dart';

import '../main.dart' show kPrimary, kDark;

const int kDefaultTurns = 6;
const String kEspWsUrl = 'ws://192.168.4.1:81';

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
	bool _sequenceStarted = false;
	bool _espConnected = false;
	String? _lastSent;
	String? _lastReceived;
	double _zoom = 1.0;
	double _minZoom = 1.0;
	double _maxZoom = 1.0;
	CameraController? _camera;
	bool _cameraReady = false;
	bool _capturing = false;
	int _primaryBackIndex = 0;
	Offset? _focusUiPos;
	Timer? _focusUiTimer;
	final _stopsCtrl = TextEditingController(text: '12');
	final List<String> _log = [];

	@override
	void initState() {
		super.initState();
		_detectBackCameras();
		_initCameraForIndex(_primaryBackIndex).then((_) => _connectWs());
	}

	@override
	void dispose() {
		_wsSub?.cancel();
		_channel?.sink.close(ws_status.normalClosure);
		_camera?.dispose();
		_stopsCtrl.dispose();
		_focusUiTimer?.cancel();
		super.dispose();
	}

	Future<void> _initCameraForIndex(int index) async {
		try {
			if (widget.cameras.isEmpty) {
				_addLog('No cameras found.');
				return;
			}
			if (index < 0 || index >= widget.cameras.length) index = 0;
			final camPerm = await Permission.camera.request();
			if (!camPerm.isGranted) {
				_addLog('Camera permission not granted.');
				return;
			}
			final desc = widget.cameras[index];
			if (_camera != null) {
				try { await _camera!.dispose(); } catch (_) {}
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
			_minZoom = await controller.getMinZoomLevel();
			_maxZoom = await controller.getMaxZoomLevel();
			final initialZoom = (_minZoom <= 1.0 && 1.0 <= _maxZoom) ? 1.0 : _minZoom;
			await controller.setZoomLevel(initialZoom);
			setState(() {
				_camera = controller;
				_cameraReady = true;
				final sliderMin = 1.0;
				_zoom = initialZoom.clamp(sliderMin, _maxZoom);
			});
			_addLog('Camera initialized (${desc.name}, ${desc.lensDirection}). min:${_minZoom.toStringAsFixed(2)} max:${_maxZoom.toStringAsFixed(2)}');
		} catch (e) {
			_addLog('Camera init error: $e');
			setState(() { _cameraReady = false; });
		}
	}

	void _detectBackCameras() {
		final backIdxs = <int>[];
		for (var i = 0; i < widget.cameras.length; i++) {
			if (widget.cameras[i].lensDirection == CameraLensDirection.back) backIdxs.add(i);
		}
		if (backIdxs.isEmpty) {
			_primaryBackIndex = 0;
			return;
		}
		_primaryBackIndex = backIdxs.first;
	}

	Future<void> _applyZoom(double value) async {
		if (_camera == null || !_cameraReady) return;
		final sliderMin = 1.0;
		value = value.clamp(sliderMin, _maxZoom);
		try {
			await _camera!.setZoomLevel(value);
			setState(() { _zoom = value; });
		} catch (e) {
			_addLog('Zoom apply error: $e');
		}
	}

	Future<void> _focusAt(Offset localPos, Size previewSize) async {
		if (_camera == null || !_cameraReady) return;
		final nx = (localPos.dx / previewSize.width).clamp(0.0, 1.0);
		final ny = (localPos.dy / previewSize.height).clamp(0.0, 1.0);
		try {
			await _camera!.setFocusPoint(Offset(nx, ny));
			await _camera!.setExposurePoint(Offset(nx, ny));
			_showFocusRing(localPos);
			_addLog('Focus at (${nx.toStringAsFixed(2)}, ${ny.toStringAsFixed(2)})');
		} catch (e) {
			_addLog('Focus error: $e');
		}
	}

	void _showFocusRing(Offset pos) {
		_focusUiTimer?.cancel();
		setState(() { _focusUiPos = pos; });
		_focusUiTimer = Timer(const Duration(milliseconds: 900), () {
			if (mounted) setState(() => _focusUiPos = null);
		});
	}

	Future<void> _connectWs() async {
		if (_connecting) return;
		setState(() { _connecting = true; });
		try {
			final channel = IOWebSocketChannel.connect(
				Uri.parse(kEspWsUrl),
				pingInterval: const Duration(seconds: 10),
			);
			_wsSub = channel.stream.listen(
				(msg) => _onWsMessage(msg.toString()),
				onDone: () {
					setState(() {
						_channel = null;
						_sequenceStarted = false;
						_espConnected = false;
					});
					_addLog('WS closed.');
				},
				onError: (err) {
					setState(() {
						_channel = null;
						_sequenceStarted = false;
						_espConnected = false;
					});
					_addLog('WS error: $err');
				},
				cancelOnError: false,
			);
			setState(() { _channel = channel; });
			_addLog('Connected to $kEspWsUrl');
		} catch (e) {
			_addLog('WS connect failed: $e');
		} finally {
			setState(() => _connecting = false);
		}
	}

	void _onWsMessage(String msg) {
		_addLog('ESP: $msg');
		setState(() { _lastReceived = msg; });
		if (msg.startsWith('CONNECTED')) {
			setState(() { _espConnected = true; });
			return;
		}
		if (msg.startsWith('STOP ')) {
			_handleStopMessage(msg);
		} else if (msg.startsWith('DONE')) {
			setState(() { _sequenceStarted = false; });
			_addLog('Sequence finished by ESP.');
		}
	}

	Future<void> _handleStopMessage(String msg) async {
		if (!_cameraReady || _capturing) return;
		setState(() => _capturing = true);
		try {
			final dir = await _photosDir();
			final stopIndex = _parseStopIndex(msg);
			final ts = DateTime.now();
			final name = 'stop_${stopIndex?.toString().padLeft(2, '0') ?? 'x'}_${_ts(ts)}.jpg';
			final savePath = '${dir.path}${Platform.pathSeparator}$name';
			_addLog('Capturing photo...');
			final XFile shot = await _camera!.takePicture();
			await File(shot.path).copy(savePath);
			_addLog('Saved: $name');
			_send('CONTINUE');
			_addLog('Sent CONTINUE');
		} catch (e) {
			_addLog('Capture error: $e');
		} finally {
			setState(() => _capturing = false);
		}
	}

	int? _parseStopIndex(String msg) {
		try {
			final parts = msg.split(' ');
			if (parts.length < 2) return null;
			final frac = parts[1].trim();
			return int.tryParse(frac.split('/').first);
		} catch (_) {
			return null;
		}
	}

	Future<Directory> _photosDir() async {
		final base = await getApplicationDocumentsDirectory();
		final dir = Directory('${base.path}${Platform.pathSeparator}ScanningTable');
		if (!await dir.exists()) { await dir.create(recursive: true); }
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
			_channel?.sink.add('$text\n');
			setState(() { _lastSent = text; });
		}
		catch (e) { _addLog('Send error: $e'); }
	}

	void _start() {
		final stops = int.tryParse(_stopsCtrl.text.trim());
		if (stops == null || stops <= 0) { _addLog('Invalid stops value.'); return; }
		if (_channel == null) { _addLog('Not connected.'); return; }
		_send('START $kDefaultTurns $stops');
		_addLog('Sent START $kDefaultTurns $stops');
		setState(() { _sequenceStarted = true; });
	}

	void _stop() {
		_send('STOP');
		_addLog('Sent STOP');
		setState(() { _sequenceStarted = false; });
	}

	@override
	Widget build(BuildContext context) {
		return Stack(
			children: [
				Container(
					decoration: const BoxDecoration(
						gradient: LinearGradient(
							colors: [kPrimary, kDark],
							begin: Alignment.topLeft,
							end: Alignment.bottomRight,
						),
					),
				),
				Scaffold(
					backgroundColor: Colors.transparent,
					body: SafeArea(
						child: Padding(
							padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 8),
							child: Column(
								crossAxisAlignment: CrossAxisAlignment.stretch,
								children: [
									Text('Scanning Table', style: Theme.of(context).textTheme.headlineLarge, textAlign: TextAlign.center),
									const SizedBox(height: 2),
									Text('ESP8266 + Camera', style: Theme.of(context).textTheme.headlineSmall?.copyWith(color: Colors.white70), textAlign: TextAlign.center),
									const SizedBox(height: 8),
									Expanded(
										flex: 6,
										child: LayoutBuilder(builder: (context, constraints) {
											final logsWidth = constraints.maxWidth * 0.30;
											final previewWidth = constraints.maxWidth - logsWidth - 12;
											return Row(
												children: [
													SizedBox(
														width: previewWidth,
														child: _previewPane(),
													),
													const SizedBox(width: 12),
													SizedBox(
														width: logsWidth,
														child: _logsPane(),
													),
												],
											);
										}),
									),
									const SizedBox(height: 8),
									Row(
										children: [
											const Icon(Icons.zoom_out, color: Colors.white),
											Expanded(
												child: Slider(
													value: _zoom,
													min: 1.0,
													max: _maxZoom <= 1.0 ? 4.0 : _maxZoom,
													divisions: 100,
													onChanged: (_cameraReady) ? (v) => _applyZoom(v) : null,
													activeColor: kPrimary,
													inactiveColor: Colors.white24,
												),
											),
											const Icon(Icons.zoom_in, color: Colors.white),
										],
									),
									const SizedBox(height: 8),
									_controlsCard(),
								],
							),
						),
					),
				),
			],
		);
	}

	Widget _previewPane() {
		return Card(
			color: Colors.white.withOpacity(0.08),
			elevation: 0,
			shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(16)),
			child: Padding(
				padding: const EdgeInsets.all(10.0),
				child: Container(
					decoration: BoxDecoration(color: Colors.black, borderRadius: BorderRadius.circular(12)),
					clipBehavior: Clip.antiAlias,
					child: _camera != null && _cameraReady
							? LayoutBuilder(
									builder: (context, cons) {
										final boxSize = Size(cons.maxWidth, cons.maxHeight);
										return Stack(
											fit: StackFit.expand,
											children: [
												Center(
													child: AspectRatio(
														aspectRatio: 9 / 16,
														child: LayoutBuilder(
															builder: (ctx, inner) {
																final innerSize = Size(inner.maxWidth, inner.maxHeight);
																return GestureDetector(
																	behavior: HitTestBehavior.opaque,
																	onTapDown: (d) => _focusAt(d.localPosition, innerSize),
																	child: CameraPreview(_camera!),
																);
															},
														),
													),
												),
												if (_focusUiPos != null)
													CustomPaint(
														painter: FocusPainter(point: _focusUiPos!),
														size: boxSize,
													),
											],
										);
									},
								)
							: const Center(child: Text('Camera not available', style: TextStyle(color: Colors.white70))),
				),
			),
		);
	}

	Widget _controlsCard() {
		return Card(
			color: Colors.white,
			elevation: 6,
			shadowColor: kDark.withOpacity(0.35),
			shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(14)),
			child: Padding(
				padding: const EdgeInsets.symmetric(horizontal: 12.0, vertical: 10.0),
				child: Column(
					children: [
						Row(
							children: [
								Expanded(
									child: TextField(
										controller: _stopsCtrl,
										keyboardType: TextInputType.number,
										decoration: InputDecoration(
											filled: true,
											fillColor: const Color(0xFFF3F6F8),
											hintText: 'Number of Stops',
											border: OutlineInputBorder(
												borderRadius: BorderRadius.circular(12),
												borderSide: BorderSide.none,
											),
											prefixIcon: const Icon(Icons.flag_rounded, color: kDark),
										),
										style: const TextStyle(fontSize: 16),
									),
								),
								const SizedBox(width: 10),
								PillButton(
									label: 'Start',
									icon: Icons.play_arrow_rounded,
									color: kPrimary,
									onPressed: (_channel != null && !_capturing) ? _start : null,
								),
							],
						),
						const SizedBox(height: 10),
						Row(
							children: [
								Expanded(
									child: ElevatedButton.icon(
										onPressed: _connecting ? null : _connectWs,
										style: ElevatedButton.styleFrom(
											backgroundColor: kDark,
											foregroundColor: Colors.white,
											padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 12),
											shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
										),
										icon: const Icon(Icons.wifi_tethering_rounded),
										label: const Text('Reconnect'),
									),
								),
								const SizedBox(width: 10),
								Expanded(
									child: ElevatedButton.icon(
										onPressed: _stop,
										style: ElevatedButton.styleFrom(
											backgroundColor: Colors.redAccent,
											foregroundColor: Colors.white,
											padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 12),
											shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
										),
										icon: const Icon(Icons.stop_rounded),
										label: const Text('Stop'),
									),
								),
							],
						),
					],
				),
			),
		);
	}

	Widget _logsPane() {
		final connected = _espConnected;
		final cameraOk = _cameraReady;
		final busy = _capturing;
		final sentStart = _sequenceStarted;
		final saved = _log.any((l) => l.contains('Saved:'));
		final hasError = _log.any((l) => l.toLowerCase().contains('error'));
		final focused = _log.any((l) => l.contains('Focus at'));

		Widget statusIconWithText(IconData icon, String label, bool on) {
			final bg = on ? kDark : Colors.white24;
			final fg = on ? Colors.white : Colors.white70;
			return Column(
				mainAxisSize: MainAxisSize.min,
				crossAxisAlignment: CrossAxisAlignment.center,
				children: [
					Tooltip(
						message: label,
						child: Container(
							width: 28,
							height: 28,
							margin: const EdgeInsets.symmetric(vertical: 2, horizontal: 4),
							child: CircleAvatar(
								backgroundColor: bg,
								radius: 12,
								child: Icon(icon, color: fg, size: 13),
							),
						),
					),
					const SizedBox(height: 2),
					Text(label, style: const TextStyle(color: Colors.white, fontSize: 10, fontWeight: FontWeight.w400)),
				],
			);
		}

		return Container(
			decoration: BoxDecoration(
				color: Colors.transparent,
				borderRadius: BorderRadius.circular(16),
			),
			padding: const EdgeInsets.fromLTRB(8, 10, 8, 10),
			child: Column(
				crossAxisAlignment: CrossAxisAlignment.center,
				children: [
					statusIconWithText(Icons.wifi, 'Connected to ESP', connected),
					statusIconWithText(Icons.photo_camera, 'Camera Ready', cameraOk),
					statusIconWithText(Icons.play_arrow, 'Sequence Started', sentStart),
					statusIconWithText(Icons.camera_alt, 'Photo Captured', saved),
					statusIconWithText(Icons.center_focus_strong, 'Focused', focused),
					statusIconWithText(Icons.hourglass_bottom, 'Capturing', busy),
					statusIconWithText(Icons.error_outline, 'Error', hasError),
					const SizedBox(height: 8),
					if (_lastSent != null)
						Text('Last sent: ${_lastSent!.length > 40 ? '${_lastSent!.substring(0,40)}...' : _lastSent}', style: const TextStyle(color: Colors.white70, fontSize: 10)),
					if (_lastReceived != null)
						Text('Last recv: ${_lastReceived!.length > 40 ? '${_lastReceived!.substring(0,40)}...' : _lastReceived}', style: const TextStyle(color: Colors.white70, fontSize: 10)),
				],
			),
		);
	}
}
