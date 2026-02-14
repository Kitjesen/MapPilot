import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:flutter_monitor/features/map/map_manager_page.dart';
import '../../test_helpers.dart';

void main() {
  group('MapManagerPage', () {
    testWidgets('renders without errors', (tester) async {
      await pumpApp(tester, const MapManagerPage());
      await tester.pump(const Duration(milliseconds: 100));

      // Page should render — check for Scaffold
      expect(find.byType(Scaffold), findsWidgets);
      expect(find.byType(MapManagerPage), findsOneWidget);
    });

    testWidgets('renders when not connected without crash', (tester) async {
      await pumpApp(tester, const MapManagerPage(), connected: false);
      await tester.pump(const Duration(milliseconds: 100));

      // Should render a page structure — not crash
      expect(find.byType(MapManagerPage), findsOneWidget);
    });
  });
}
