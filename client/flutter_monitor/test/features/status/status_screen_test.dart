import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:flutter_monitor/features/status/status_screen.dart';
import '../../test_helpers.dart';

void main() {
  group('StatusScreen', () {
    testWidgets('renders without errors when disconnected', (tester) async {
      // Use a larger surface to avoid overflow in test
      tester.view.physicalSize = const Size(1080, 1920);
      tester.view.devicePixelRatio = 1.0;
      addTearDown(() {
        tester.view.resetPhysicalSize();
        tester.view.resetDevicePixelRatio();
      });

      await pumpApp(tester, const StatusScreen(), connected: false);
      await tester.pump(const Duration(milliseconds: 100));

      // Should render — not crash
      expect(find.byType(StatusScreen), findsOneWidget);
      expect(find.byType(Scaffold), findsWidgets);
    });

    testWidgets('widget type is correct', (tester) async {
      tester.view.physicalSize = const Size(1080, 1920);
      tester.view.devicePixelRatio = 1.0;
      addTearDown(() {
        tester.view.resetPhysicalSize();
        tester.view.resetDevicePixelRatio();
      });

      await pumpApp(tester, const StatusScreen());
      await tester.pump(const Duration(milliseconds: 100));

      expect(find.byType(StatusScreen), findsOneWidget);
    });
  });
}
