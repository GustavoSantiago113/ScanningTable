import 'package:camera/camera.dart';
import 'package:flutter/material.dart';
import 'package:http/http.dart' as http;

import '../main.dart' show kPrimary, kDark, kEspBaseUrl, kEspSsid;
import 'scanning_page.dart';

class InitialPage extends StatefulWidget {
  final List<CameraDescription> cameras;
  const InitialPage({super.key, required this.cameras});

  @override
  State<InitialPage> createState() => _InitialPageState();
}

class _InitialPageState extends State<InitialPage> {
  bool _connecting = false;
  String? _errorMsg;

  Future<void> _connect() async {
    if (_connecting) return;
    setState(() {
      _connecting = true;
      _errorMsg = null;
    });
    try {
      final resp = await http
          .get(Uri.parse('$kEspBaseUrl/status'))
          .timeout(const Duration(seconds: 5));
      if (!mounted) return;
      if (resp.statusCode == 200) {
        Navigator.of(context).push(MaterialPageRoute(
          builder: (_) => ScanningPage(cameras: widget.cameras),
        ));
      } else {
        setState(() {
          _errorMsg = 'ESP responded with status ${resp.statusCode}.';
        });
      }
    } catch (_) {
      if (mounted) {
        setState(() {
          _errorMsg =
              'Could not reach ESP.\nMake sure you are connected to "$kEspSsid" Wi-Fi.';
        });
      }
    } finally {
      if (mounted) setState(() => _connecting = false);
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.white,
      body: Center(
        child: Padding(
          padding: const EdgeInsets.symmetric(horizontal: 32),
          child: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              Icon(Icons.document_scanner_rounded, size: 80, color: kPrimary),
              const SizedBox(height: 24),
              Text(
                'Scanning Table',
                style: Theme.of(context).textTheme.headlineLarge,
              ),
              const SizedBox(height: 8),
              Text(
                'Connect to the "$kEspSsid" Wi-Fi network before continuing.',
                textAlign: TextAlign.center,
                style: const TextStyle(color: Colors.black54, fontSize: 14),
              ),
              const SizedBox(height: 40),
              SizedBox(
                width: double.infinity,
                child: ElevatedButton.icon(
                  onPressed: _connecting ? null : _connect,
                  style: ElevatedButton.styleFrom(
                    backgroundColor: kPrimary,
                    foregroundColor: Colors.white,
                    padding: const EdgeInsets.symmetric(vertical: 16),
                    shape: RoundedRectangleBorder(
                        borderRadius: BorderRadius.circular(16)),
                  ),
                  icon: _connecting
                      ? const SizedBox(
                          width: 20,
                          height: 20,
                          child: CircularProgressIndicator(
                              strokeWidth: 2, color: Colors.white),
                        )
                      : const Icon(Icons.wifi_rounded),
                  label: Text(
                    _connecting ? 'Connecting…' : 'Connect',
                    style: const TextStyle(
                        fontSize: 16, fontWeight: FontWeight.w700),
                  ),
                ),
              ),
              if (_errorMsg != null) ...[
                const SizedBox(height: 16),
                Text(
                  _errorMsg!,
                  textAlign: TextAlign.center,
                  style: const TextStyle(color: Colors.redAccent, fontSize: 13),
                ),
              ],
            ],
          ),
        ),
      ),
    );
  }
}
