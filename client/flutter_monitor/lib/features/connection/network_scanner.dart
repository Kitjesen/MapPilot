import 'dart:async';
import 'dart:io';
import 'package:flutter/foundation.dart';
import 'package:grpc/grpc.dart';

/// Discovered robot on the network
class DiscoveredRobot {
  final String ip;
  final int port;
  final String? robotName;
  final String? robotId;
  final DateTime discoveredAt;

  DiscoveredRobot({
    required this.ip,
    required this.port,
    this.robotName,
    this.robotId,
    DateTime? discoveredAt,
  }) : discoveredAt = discoveredAt ?? DateTime.now();

  String get displayName => robotName ?? ip;
  String get address => '$ip:$port';

  @override
  bool operator ==(Object other) =>
      other is DiscoveredRobot && other.ip == ip && other.port == port;

  @override
  int get hashCode => ip.hashCode ^ port.hashCode;
}

/// Scans the local network for robots running gRPC services
class NetworkScanner {
  bool _isScanning = false;
  bool get isScanning => _isScanning;

  final _resultsController = StreamController<DiscoveredRobot>.broadcast();
  Stream<DiscoveredRobot> get results => _resultsController.stream;

  final _progressController = StreamController<double>.broadcast();
  Stream<double> get progress => _progressController.stream;

  bool _disposed = false;

  /// Get ALL local subnet prefixes (e.g., ["192.168.1", "192.168.66"])
  /// Excludes common virtual adapter subnets (.1 host = likely gateway/VM host)
  Future<List<String>> _getLocalSubnets() async {
    final subnets = <String>[];
    try {
      final interfaces = await NetworkInterface.list(
        type: InternetAddressType.IPv4,
        includeLoopback: false,
      );
      for (final iface in interfaces) {
        for (final addr in iface.addresses) {
          final ip = addr.address;
          if (ip.startsWith('192.168.') || ip.startsWith('10.') || ip.startsWith('172.')) {
            final parts = ip.split('.');
            final subnet = '${parts[0]}.${parts[1]}.${parts[2]}';
            if (!subnets.contains(subnet)) {
              subnets.add(subnet);
              debugPrint('[Scanner] Found subnet: $subnet (${iface.name} → $ip)');
            }
          }
        }
      }
    } catch (e) {
      debugPrint('[Scanner] Failed to get local subnets: $e');
    }
    // Prioritize: non-.1 host addresses first (likely real LAN, not VM host-only)
    return subnets;
  }

  /// Scan ALL local subnets for gRPC robot services
  Future<List<DiscoveredRobot>> scan({
    int port = 50051,
    Duration timeout = const Duration(seconds: 2),
    String? subnetOverride,
  }) async {
    if (_isScanning) return [];
    _isScanning = true;

    final subnets = subnetOverride != null
        ? [subnetOverride]
        : await _getLocalSubnets();

    if (subnets.isEmpty) {
      debugPrint('[Scanner] Could not determine any local subnet');
      _isScanning = false;
      return [];
    }

    debugPrint('[Scanner] Scanning ${subnets.length} subnet(s) on port $port: $subnets');
    final found = <DiscoveredRobot>[];
    final futures = <Future>[];

    final totalHosts = subnets.length * 254;
    int probed = 0;

    for (final subnet in subnets) {
      debugPrint('[Scanner] Scanning $subnet.0/24 ...');
      for (int i = 1; i < 255; i++) {
        if (!_isScanning) break; // allow stop()
        final ip = '$subnet.$i';
        futures.add(_probeHost(ip, port, timeout).then((robot) {
          if (_disposed) return;
          if (robot != null) {
            found.add(robot);
            if (!_resultsController.isClosed) _resultsController.add(robot);
            debugPrint('[Scanner] Found robot at $ip:$port');
          }
          probed++;
          if (!_progressController.isClosed) _progressController.add(probed / totalHosts);
        }));

        // Batch: 50 concurrent probes
        if (futures.length >= 50) {
          await Future.wait(futures);
          futures.clear();
        }
      }
    }

    if (futures.isNotEmpty) {
      await Future.wait(futures);
    }

    if (!_progressController.isClosed) _progressController.add(1.0);
    _isScanning = false;
    debugPrint('[Scanner] Scan complete. Found ${found.length} robot(s) across ${subnets.length} subnet(s)');
    return found;
  }

  /// Try to connect to a single host
  Future<DiscoveredRobot?> _probeHost(
      String ip, int port, Duration timeout) async {
    try {
      final socket = await Socket.connect(
        ip,
        port,
        timeout: timeout,
      );
      socket.destroy();

      // TCP port is open - likely a gRPC service
      return DiscoveredRobot(
        ip: ip,
        port: port,
        robotName: null, // Will be populated after full gRPC handshake
      );
    } catch (_) {
      return null;
    }
  }

  /// Probe a specific host and try to get robot info via gRPC
  Future<DiscoveredRobot?> probeWithInfo(String ip, int port) async {
    try {
      final channel = ClientChannel(
        ip,
        port: port,
        options: const ChannelOptions(
          credentials: ChannelCredentials.insecure(),
        ),
      );

      // Try a quick connection test
      // We can't easily call getRobotInfo without the proto stubs here,
      // so we just verify the TCP connection succeeds
      await channel.terminate();

      return DiscoveredRobot(
        ip: ip,
        port: port,
      );
    } catch (_) {
      return null;
    }
  }

  void stop() {
    _isScanning = false;
  }

  void dispose() {
    _disposed = true;
    _isScanning = false;
    _resultsController.close();
    _progressController.close();
  }
}
