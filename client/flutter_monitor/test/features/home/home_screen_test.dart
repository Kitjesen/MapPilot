import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:flutter_monitor/features/home/home_screen.dart';
import '../../test_helpers.dart';

void main() {
  group('HomeScreen', () {
    testWidgets('renders branded header text', (tester) async {
      // Provide routes that HomeScreen might navigate to
      await pumpApp(
        tester,
        const HomeScreen(),
        routes: {
          '/scan': (_) => const Scaffold(),
          '/robot-select': (_) => const Scaffold(),
          '/robot-detail': (_) => const Scaffold(),
        },
      );

      // The branded header uses "大算" and "NAV" in a Text.rich
      // find.textContaining finds text within RichText / Text.rich
      expect(find.textContaining('大算'), findsWidgets);
    });

    testWidgets('renders without crashing when disconnected', (tester) async {
      await pumpApp(
        tester,
        const HomeScreen(),
        connected: false,
        routes: {
          '/scan': (_) => const Scaffold(),
          '/robot-select': (_) => const Scaffold(),
        },
      );
      await tester.pump(const Duration(milliseconds: 100));

      // Widget tree rendered without error
      expect(find.byType(HomeScreen), findsOneWidget);
    });
  });
}
