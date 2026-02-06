import 'dart:async';
import 'dart:typed_data';
import 'package:flutter/material.dart';
import '../services/robot_client_base.dart';
import 'package:robot_proto/src/common.pb.dart';
import 'package:robot_proto/src/data.pb.dart';

class CameraStreamWidget extends StatefulWidget {
  final RobotClientBase client;

  const CameraStreamWidget({super.key, required this.client});

  @override
  State<CameraStreamWidget> createState() => _CameraStreamWidgetState();
}

class _CameraStreamWidgetState extends State<CameraStreamWidget> {
  StreamSubscription<DataChunk>? _cameraSubscription;
  Uint8List? _currentImageBytes;
  bool _hasError = false;
  String _errorMessage = 'No Signal';
  DateTime? _lastFrameTime;

  @override
  void initState() {
    super.initState();
    _subscribeToCamera();
  }

  @override
  void dispose() {
    _cameraSubscription?.cancel();
    super.dispose();
  }

  void _subscribeToCamera() {
    try {
      final resourceId = ResourceId()
        ..type = ResourceType.RESOURCE_TYPE_CAMERA
        // Use Orbbec color compressed topic directly; backend still supports parameter fallback.
        ..name = '/camera/color/image_raw/compressed';

      _cameraSubscription = widget.client.subscribeToResource(resourceId).listen(
        (chunk) {
          if (chunk.data.isNotEmpty) {
            setState(() {
              _currentImageBytes = Uint8List.fromList(chunk.data);
              _hasError = false;
              _lastFrameTime = DateTime.now();
            });
          }
        },
        onError: (error) {
          setState(() {
            _hasError = true;
            _errorMessage = 'Stream Error: $error';
          });
        },
      );
    } catch (e) {
      setState(() {
        _hasError = true;
        _errorMessage = 'Failed to connect: $e';
      });
    }
  }

  @override
  Widget build(BuildContext context) {
    // Check if frame is stale (no updates in last 5 seconds)
    final now = DateTime.now();
    final isStale = _lastFrameTime != null && 
                    now.difference(_lastFrameTime!).inSeconds > 5;

    if (_currentImageBytes == null || _currentImageBytes!.isEmpty || _hasError || isStale) {
      return _buildPlaceholder();
    }

    return Image.memory(
      _currentImageBytes!,
      fit: BoxFit.cover,
      gaplessPlayback: true,
      errorBuilder: (context, error, stackTrace) {
        return _buildPlaceholder();
      },
    );
  }

  Widget _buildPlaceholder() {
    return Container(
      decoration: BoxDecoration(
        gradient: LinearGradient(
          begin: Alignment.topLeft,
          end: Alignment.bottomRight,
          colors: [
            Colors.grey[900]!,
            Colors.grey[800]!,
            Colors.grey[900]!,
          ],
        ),
      ),
      child: Center(
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            Icon(
              Icons.videocam_off,
              size: 64,
              color: Colors.white.withOpacity(0.3),
            ),
            const SizedBox(height: 16),
            Text(
              _hasError ? _errorMessage : 'Simulation Mode',
              style: TextStyle(
                color: Colors.white.withOpacity(0.5),
                fontSize: 16,
                fontWeight: FontWeight.w500,
              ),
            ),
            const SizedBox(height: 8),
            Text(
              'No Camera Feed',
              style: TextStyle(
                color: Colors.white.withOpacity(0.3),
                fontSize: 12,
              ),
            ),
          ],
        ),
      ),
    );
  }
}
