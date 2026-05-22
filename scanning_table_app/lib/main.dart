import 'package:flutter/material.dart';
import 'package:camera/camera.dart';
import 'package:google_fonts/google_fonts.dart';
import 'pages/pages.dart';

const kPrimary = Color(0xFF05989E);
const kDark = Color(0xFF3C444B);
const kEspBaseUrl = 'http://192.168.4.1';
const kEspSsid = 'ESP_Motor_WS';

Future<void> main() async {
  WidgetsFlutterBinding.ensureInitialized();
  final cameras = await availableCameras();
  runApp(ScanningTableApp(cameras: cameras));
}

class ScanningTableApp extends StatelessWidget {
  final List<CameraDescription> cameras;
  const ScanningTableApp({super.key, required this.cameras});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      debugShowCheckedModeBanner: false,
      title: 'Scanning Table',
      theme: ThemeData(
        colorScheme: ColorScheme.fromSeed(seedColor: kPrimary),
        useMaterial3: true,
        scaffoldBackgroundColor: Colors.white,
        textTheme: GoogleFonts.crimsonTextTextTheme(Theme.of(context).textTheme).copyWith(
          headlineLarge: GoogleFonts.crimsonText(fontSize: 28, fontWeight: FontWeight.bold, color: kDark),
          headlineSmall: GoogleFonts.crimsonText(fontSize: 18, fontWeight: FontWeight.w500, color: kDark),
        ),
      ),
      home: InitialPage(cameras: cameras),
    );
  }
}