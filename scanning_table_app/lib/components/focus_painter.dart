import 'package:flutter/material.dart';

class FocusPainter extends CustomPainter {
  final Offset point;
  FocusPainter({required this.point});

  @override
  void paint(Canvas canvas, Size size) {
    final p = point;
    final paint = Paint()
      ..color = Colors.white
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2;
    const r = 28.0;
    canvas.drawCircle(p, r, paint);
    // small crosshair
    canvas.drawLine(Offset(p.dx - r, p.dy), Offset(p.dx - r / 2, p.dy), paint);
    canvas.drawLine(Offset(p.dx + r / 2, p.dy), Offset(p.dx + r, p.dy), paint);
    canvas.drawLine(Offset(p.dx, p.dy - r), Offset(p.dx, p.dy - r / 2), paint);
    canvas.drawLine(Offset(p.dx, p.dy + r / 2), Offset(p.dx, p.dy + r), paint);
  }

  @override
  bool shouldRepaint(covariant FocusPainter oldDelegate) => oldDelegate.point != point;
}
