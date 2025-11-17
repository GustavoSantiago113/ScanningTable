import 'dart:async';
import 'dart:io';
import 'dart:convert';
import 'package:camera/camera.dart';
import 'package:flutter/material.dart';
import 'package:path_provider/path_provider.dart';
import 'package:http/http.dart' as http;
import 'package:permission_handler/permission_handler.dart';
import 'package:wakelock_plus/wakelock_plus.dart';
import 'package:flutter/services.dart';
import '../components/components.dart';

import '../main.dart' show kPrimary, kDark;

const int kDefaultTurns = 6;
const String kEspBaseUrl = 'http://192.168.4.1';
const Duration kCaptureDelay = Duration(milliseconds: 800); // wait after motor stops

class HomeScreen extends StatefulWidget {
	final List<CameraDescription> cameras;
	const HomeScreen({super.key, required this.cameras});

	@override
	State<HomeScreen> createState() => _HomeScreenState();
}

class _HomeScreenState extends State<HomeScreen> {
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
	final MethodChannel _safChannel = const MethodChannel('com.gustavo.saf');
	String? _safTreeUri;
	Offset? _focusUiPos;
	Timer? _focusUiTimer;
	final _stopsCtrl = TextEditingController(text: '12');
	final List<String> _log = [];
	bool _flashOn = false;
	Future<void> _toggleFlash(bool value) async {
		if (_camera == null || !_cameraReady) return;
		try {
			await _camera!.setFlashMode(value ? FlashMode.torch : FlashMode.off);
			setState(() {
				_flashOn = value;
			});
		} catch (e) {
			_addLog('Flash error: $e');
		}
	}

	@override
	void initState() {
		super.initState();
		// Keep screen awake while using the app
		WakelockPlus.enable();
		_detectBackCameras();
		// Prepare storage folder early and then init camera & ping
		_ensurePhotoDir();
		_initCameraForIndex(_primaryBackIndex).then((_) => _ping());
	}

	Timer? _pollTimer;
	int _lastProcessedStop = 0; // to avoid double-processing same stop

	@override
	void dispose() {
		// Allow screen to sleep again when leaving the app
		WakelockPlus.disable();
		_pollTimer?.cancel();
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
				ResolutionPreset.max, // Use the highest available resolution
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

	Future<void> _handleStopEvent(int stopIndex) async {
		if (!_cameraReady || _capturing) return;
		setState(() => _capturing = true);
		try {
			final dir = await _photosDir();
			final ts = DateTime.now();
			final name = 'stop_${stopIndex.toString().padLeft(2, '0')}_${_ts(ts)}.jpg';
			final savePath = '${dir.path}${Platform.pathSeparator}$name';
			_addLog('Capturing photo...');
			// Take picture with highest quality (if supported)
			final XFile shot = await _camera!.takePicture();
			final bytes = await File(shot.path).readAsBytes();
			// If the user picked a SAF folder, try saving via the native SAF bridge
			if (_safTreeUri != null) {
				try {
					final b64 = base64Encode(bytes);
					final ok = await _safChannel.invokeMethod<bool>('saveFileToDirectory', {
						'treeUri': _safTreeUri,
						'filename': name,
						'base64': b64,
					});
					if (ok == true) {
						_addLog('Saved (SAF): $name');
						if (mounted) ScaffoldMessenger.of(context).showSnackBar(
							SnackBar(content: Text('Saved $name'), duration: const Duration(seconds: 2), action: SnackBarAction(label: 'Open', onPressed: _openDownloads)),
						);
					} else {
						// Fallback to local copy
						await File(shot.path).copy(savePath);
						_addLog('Saved: $name');
					}
				} catch (e) {
					_addLog('SAF save failed: $e');
					// Fallback
					await File(shot.path).copy(savePath);
					_addLog('Saved: $name');
				}
			} else {
				await File(shot.path).copy(savePath);
				_addLog('Saved: $name');
				// Notify the media scanner so the image appears in gallery
				if (Platform.isAndroid) {
					try {
						await Process.run('am', [
							'broadcast',
							'-a',
							'android.intent.action.MEDIA_SCANNER_SCAN_FILE',
							'-d',
							'file://$savePath'
						]);
					} catch (e) {
						// Ignore if media scanner notification fails
					}
				}
			}
			// Give the device a short moment to ensure file IO settles before continuing the motor
			await Future.delayed(kCaptureDelay);
			await _httpContinue();
			_addLog('Sent CONTINUE');
		} catch (e) {
			_addLog('Capture error: $e');
		} finally {
			setState(() => _capturing = false);
		}
	}

	Future<Directory> _photosDir() async {
		// Save images to the Android Downloads folder when possible to make them easy to find.
		try {
			if (Platform.isAndroid) {
				// Request storage permission on Android
				final storagePermission = await Permission.storage.request();
				if (!storagePermission.isGranted) {
					_addLog('Storage permission not granted, falling back to app directory');
					final appDir = await getApplicationDocumentsDirectory();
					final fallback = Directory('${appDir.path}${Platform.pathSeparator}ScanningTable');
					if (!await fallback.exists()) await fallback.create(recursive: true);
					return fallback;
				}
				// Common public Downloads path on Android
				final downloads = Directory('/storage/emulated/0/Download');
				if (!await downloads.exists()) await downloads.create(recursive: true);
				return downloads;
			} else {
				// Non-Android: use app documents directory
				final appDir = await getApplicationDocumentsDirectory();
				final dir = Directory('${appDir.path}${Platform.pathSeparator}ScanningTable');
				if (!await dir.exists()) await dir.create(recursive: true);
				return dir;
			}
		} catch (e) {
			_addLog('Error selecting Downloads directory: $e');
			final appDir = await getApplicationDocumentsDirectory();
			final fallback = Directory('${appDir.path}${Platform.pathSeparator}ScanningTable');
			if (!await fallback.exists()) await fallback.create(recursive: true);
			return fallback;
		}
	}

	Future<void> _ensurePhotoDir() async {
		try {
			final dir = await _photosDir();
			_addLog('Photos dir: ${dir.path}');
		} catch (e) {
			_addLog('Photo dir error: $e');
		}
	}

	Future<void> _pickFolder() async {
		try {
			final uri = await _safChannel.invokeMethod<String>('pickDirectory');
			if (uri != null) {
				setState(() => _safTreeUri = uri);
				_addLog('Picked SAF folder: $uri');
				if (mounted) ScaffoldMessenger.of(context).showSnackBar(const SnackBar(content: Text('Folder selected')));
			}
		} catch (e) {
			_addLog('Pick folder failed: $e');
		}
	}

	Future<void> _openDownloads() async {
		try {
			await _safChannel.invokeMethod('openDownloads');
		} catch (e) {
			_addLog('Open folder failed: $e');
		}
	}

	String _ts(DateTime dt) {
		String two(int n) => n.toString().padLeft(2, '0');
		return '${dt.year}${two(dt.month)}${two(dt.day)}_${two(dt.hour)}${two(dt.minute)}${two(dt.second)}';
	}

	void _addLog(String line) {
		setState(() => _log.insert(0, '${DateTime.now().toIso8601String().substring(11, 19)}  $line'));
	}

	void _start() {
		final stops = int.tryParse(_stopsCtrl.text.trim());
		if (stops == null || stops <= 0) { _addLog('Invalid stops value.'); return; }
		_httpStart(kDefaultTurns, stops);
	}

	void _stop() {
		_httpStop();
	}

	// ------------------ HTTP layer ------------------
	Uri _uri(String path, [Map<String, String>? q]) => Uri.parse('$kEspBaseUrl$path').replace(queryParameters: q);

	Future<void> _httpStart(int turns, int stops) async {
			try {
				setState(() { _sequenceStarted = true; _lastProcessedStop = 0; });
				final resp = await http
						.post(_uri('/start', { 'turns': '$turns', 'stops': '$stops' }))
						.timeout(const Duration(seconds: 5));
			_addLog('HTTP /start => ${resp.statusCode} ${resp.body}');
			setState(() { _espConnected = resp.statusCode == 200; _lastSent = 'START $turns $stops'; });
			_startPolling();
		} catch (e) {
			_addLog('HTTP start failed: $e');
			setState(() { _sequenceStarted = false; _espConnected = false; });
		}
	}

	Future<void> _httpStop() async {
			try {
				final resp = await http.post(_uri('/stop')).timeout(const Duration(seconds: 5));
			_addLog('HTTP /stop => ${resp.statusCode}');
			setState(() { _sequenceStarted = false; _lastSent = 'STOP'; });
		} catch (e) {
			_addLog('HTTP stop failed: $e');
		}
	}

	Future<void> _httpContinue() async {
			try {
				final resp = await http.post(_uri('/continue')).timeout(const Duration(seconds: 5));
			_addLog('HTTP /continue => ${resp.statusCode}');
			setState(() { _lastSent = 'CONTINUE'; });
		} catch (e) {
			_addLog('HTTP continue failed: $e');
		}
	}

	Future<void> _ping() async {
		if (_connecting) return;
		setState(() { _connecting = true; });
			try {
				final resp = await http.get(_uri('/status')).timeout(const Duration(seconds: 5));
			setState(() { _espConnected = resp.statusCode == 200; _lastReceived = resp.body; });
			_addLog('HTTP /status => ${resp.statusCode}');
		} catch (e) {
			setState(() { _espConnected = false; });
            _addLog('Ping failed: ${e.runtimeType}: $e');
		} finally {
			setState(() { _connecting = false; });
		}
	}

	void _startPolling() {
		_pollTimer?.cancel();
		_pollTimer = Timer.periodic(const Duration(milliseconds: 400), (_) async {
			try {
				final resp = await http.get(_uri('/status')).timeout(const Duration(seconds: 5));
				if (resp.statusCode != 200) return;
				final data = jsonDecode(resp.body) as Map<String, dynamic>;
				final running = data['running'] == true;
				final waiting = data['waitingForContinue'] == true;
				final current = (data['currentStop'] ?? 0) as int;
				final total = (data['totalStops'] ?? 0) as int;
				setState(() {
					_espConnected = true;
					_sequenceStarted = running;
					_lastReceived = resp.body;
				});
				if (waiting && current > _lastProcessedStop) {
					_lastProcessedStop = current;
					await _handleStopEvent(current);
				}
				if (!running && total > 0 && current >= total) {
					_addLog('Sequence finished by ESP.');
					_pollTimer?.cancel();
					setState(() { _sequenceStarted = false; });
				}
			} catch (_) {
				// ignore transient errors while polling
			}
		});
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
													onChanged: (_cameraReady && _safTreeUri != null) ? (v) => _applyZoom(v) : null,
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
									onPressed: (_safTreeUri != null && _espConnected && !_capturing && !_sequenceStarted) ? _start : null,
								),
							],
						),
						const SizedBox(height: 10),
						Row(
							children: [
								Expanded(
									child: ElevatedButton.icon(
										onPressed: (_connecting || _safTreeUri == null) ? null : _ping,
										style: ElevatedButton.styleFrom(
											backgroundColor: kDark,
											foregroundColor: Colors.white,
											padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 12),
											shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
										),
										icon: const Icon(Icons.wifi_tethering_rounded),
										label: const Text('Ping'),
									),
								),
								const SizedBox(width: 10),
								Expanded(
									child: ElevatedButton.icon(
										onPressed: (_safTreeUri != null && _sequenceStarted) ? _stop : null,
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
						const SizedBox(height: 10),
						Row(
							mainAxisAlignment: MainAxisAlignment.center,
							children: [
								ElevatedButton.icon(
									onPressed: _pickFolder,
									style: ElevatedButton.styleFrom(
										backgroundColor: kPrimary,
										foregroundColor: Colors.white,
										padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 12),
										shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
									),
									icon: const Icon(Icons.folder_open),
									label: const Text('Pick folder'),
								),
								const SizedBox(width: 12),
								Row(
									children: [
										const Icon(Icons.flash_on, color: kDark),
										Switch(
											value: _flashOn,
											onChanged: (_cameraReady && _camera != null)
													? (v) => _toggleFlash(v)
													: null,
											activeThumbColor: kPrimary,
										),
									],
								),
								const SizedBox(width: 12),
								if (_safTreeUri == null)
									Flexible(
										child: Text('Please select a folder before starting.', style: TextStyle(color: Colors.redAccent, fontWeight: FontWeight.w600)),
									),
							],
						),
          ]
        ),
      )
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
						child: Scrollbar(
							thumbVisibility: true,
							child: ListView(
								padding: EdgeInsets.zero,
								children: [
									statusIconWithText(Icons.wifi, 'Connected to ESP', connected),
									statusIconWithText(Icons.photo_camera, 'Camera Ready', cameraOk),
									statusIconWithText(Icons.play_arrow, 'Sequence Started', sentStart),
									statusIconWithText(Icons.camera_alt, 'Photo Captured', saved),
									statusIconWithText(Icons.center_focus_strong, 'Focused', focused),
									statusIconWithText(Icons.hourglass_bottom, 'Capturing', busy),
									statusIconWithText(Icons.error_outline, 'Error', hasError),
								],
							),
						),
					);
		}
}
