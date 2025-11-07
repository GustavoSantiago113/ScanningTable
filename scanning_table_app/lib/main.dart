import 'package:flutter/material.dart';
import 'package:camera/camera.dart';
import 'pages/pages.dart';

const kPrimary = Color(0xFF05989E);
const kDark = Color(0xFF3C444B);

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
        scaffoldBackgroundColor: kDark,
        textTheme: const TextTheme(
          headlineLarge: TextStyle(fontWeight: FontWeight.bold, color: Colors.white, fontSize: 28),
          headlineSmall: TextStyle(fontWeight: FontWeight.w500, color: Colors.white, fontSize: 18),
        ),
      ),  
      home: HomeScreen(cameras: cameras),
    );
  }
}