import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:flutter_monitor/features/connection/splash_screen.dart';
import '../../test_helpers.dart';

void main() {
  group('SplashScreen', () {
    testWidgets('renders brand logo and tagline', (tester) async {
      // Use a MaterialApp with /main route to prevent navigation errors
      await pumpApp(
        tester,
        const SplashScreen(),
        routes: {'/main': (_) => const Scaffold(body: Text('Main'))},
      );

      // Brand monogram "DS"
      expect(find.text('DS'), findsOneWidget);

      // App name
      expect(find.text('大算机器人'), findsOneWidget);

      // Tagline
      expect(find.text('让每一台机器人都拥有智慧'), findsOneWidget);

      // Version number
      expect(find.text('v$kAppVersion'), findsOneWidget);

      // Let timers complete to avoid pending timer teardown warnings
      await tester.pump(const Duration(seconds: 3));
      await tester.pumpAndSettle();
    });

    testWidgets('kAppVersion is a valid semver string', (tester) async {
      final parts = kAppVersion.split('.');
      expect(parts.length, greaterThanOrEqualTo(2));
      for (final p in parts) {
        expect(int.tryParse(p), isNotNull,
            reason: 'Each part "$p" should be a number');
      }
    });

    testWidgets('navigates to /main after delay', (tester) async {
      bool navigated = false;

      await pumpApp(
        tester,
        const SplashScreen(),
        routes: {
          '/main': (_) {
            navigated = true;
            return const Scaffold(body: Text('Main'));
          },
        },
      );

      // Before timeout — still on splash
      expect(navigated, isFalse);

      // Advance past the 1800ms delay + animation settle
      await tester.pump(const Duration(milliseconds: 2000));
      await tester.pumpAndSettle();

      // Should have navigated
      expect(navigated, isTrue);
    });
  });
}
