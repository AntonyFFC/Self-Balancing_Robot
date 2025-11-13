import 'dart:async';
import 'dart:convert';
import 'dart:io' show RawDatagramSocket, InternetAddress, File, RawSocketEvent;

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:path_provider/path_provider.dart';

void main() {
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Robot Control',
      theme: ThemeData(colorScheme: ColorScheme.fromSeed(seedColor: Colors.deepPurple)),
      home: const RobotControlPage(),
    );
  }
}

// Simple telemetry chart that draws two lines (pitch and setPitch) using CustomPainter.
class TelemetryChart extends StatelessWidget {
  final List<double> pitchBuffer;
  final List<double> setPitchBuffer;
  final String title;

  const TelemetryChart({Key? key, required this.pitchBuffer, required this.setPitchBuffer, this.title = ''}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        if (title.isNotEmpty)
          Padding(
            padding: const EdgeInsets.only(bottom: 4.0, left: 4.0),
            child: Text(title, style: Theme.of(context).textTheme.bodyMedium),
          ),
        Expanded(
          child: LayoutBuilder(
            builder: (context, constraints) {
              return CustomPaint(
                size: Size(constraints.maxWidth, constraints.maxHeight),
                painter: _TelemetryPainter(pitch: pitchBuffer, setPitch: setPitchBuffer),
              );
            },
          ),
        ),
      ],
    );
  }
}

class _TelemetryPainter extends CustomPainter {
  final List<double> pitch;
  final List<double> setPitch;

  _TelemetryPainter({required this.pitch, required this.setPitch});

  @override
  void paint(Canvas canvas, Size size) {
    final paintGrid = Paint()..color = Colors.grey.withOpacity(0.25)..strokeWidth = 1.0;
    final paintPitch = Paint()..color = Colors.blue..style = PaintingStyle.stroke..strokeWidth = 2.0;
    final paintSet = Paint()..color = Colors.red..style = PaintingStyle.stroke..strokeWidth = 2.0;

    // draw simple grid lines
    const gridLines = 4;
    for (var i = 0; i <= gridLines; i++) {
      final y = size.height * i / gridLines;
      canvas.drawLine(Offset(0, y), Offset(size.width, y), paintGrid);
    }

    final combined = <double>[];
    combined.addAll(pitch);
    combined.addAll(setPitch);
    double minY = combined.isNotEmpty ? combined.reduce((a, b) => a < b ? a : b) : 0.0;
    double maxY = combined.isNotEmpty ? combined.reduce((a, b) => a > b ? a : b) : 1.0;
    if ((maxY - minY).abs() < 1e-6) {
      maxY += 1.0;
      minY -= 1.0;
    }

    void drawLine(List<double> data, Paint p) {
      if (data.isEmpty) return;
      final path = Path();
      final n = data.length;
      for (var i = 0; i < n; i++) {
        final x = (n == 1) ? size.width / 2 : (i * size.width / (n - 1));
        final y = size.height - ((data[i] - minY) / (maxY - minY)) * size.height;
        if (i == 0) path.moveTo(x, y);
        else path.lineTo(x, y);
      }
      canvas.drawPath(path, p);
    }

    drawLine(pitch, paintPitch);
    drawLine(setPitch, paintSet);
  }

  @override
  bool shouldRepaint(covariant _TelemetryPainter oldDelegate) {
    return oldDelegate.pitch != pitch || oldDelegate.setPitch != setPitch;
  }
}

// Simple single-line control chart for u values.
class ControlChart extends StatelessWidget {
  final List<double> uBuffer;
  final String title;

  const ControlChart({Key? key, required this.uBuffer, this.title = ''}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.start,
      children: [
        if (title.isNotEmpty)
          Padding(
            padding: const EdgeInsets.only(bottom: 4.0, left: 4.0),
            child: Text(title, style: Theme.of(context).textTheme.bodyMedium),
          ),
        Expanded(
          child: LayoutBuilder(
            builder: (context, constraints) {
              return CustomPaint(
                size: Size(constraints.maxWidth, constraints.maxHeight),
                painter: _ControlPainter(u: uBuffer),
              );
            },
          ),
        ),
      ],
    );
  }
}

class _ControlPainter extends CustomPainter {
  final List<double> u;

  _ControlPainter({required this.u});

  @override
  void paint(Canvas canvas, Size size) {
    final paintGrid = Paint()..color = Colors.grey.withOpacity(0.25)..strokeWidth = 1.0;
    final paintU = Paint()..color = Colors.green..style = PaintingStyle.stroke..strokeWidth = 2.0;

    const gridLines = 3;
    for (var i = 0; i <= gridLines; i++) {
      final y = size.height * i / gridLines;
      canvas.drawLine(Offset(0, y), Offset(size.width, y), paintGrid);
    }

    double minY = 0.0;
    double maxY = 255.0;
    if (u.isNotEmpty) {
      final minData = u.reduce((a, b) => a < b ? a : b);
      final maxData = u.reduce((a, b) => a > b ? a : b);
      minY = minData < minY ? minData : minY;
      maxY = maxData > maxY ? maxData : maxY;
      if ((maxY - minY).abs() < 1e-6) maxY = minY + 1.0;
    }

    if (u.isEmpty) return;
    final path = Path();
    final n = u.length;
    for (var i = 0; i < n; i++) {
      final x = (n == 1) ? size.width / 2 : (i * size.width / (n - 1));
      final y = size.height - ((u[i] - minY) / (maxY - minY)) * size.height;
      if (i == 0) path.moveTo(x, y);
      else path.lineTo(x, y);
    }
    canvas.drawPath(path, paintU);
  }

  @override
  bool shouldRepaint(covariant _ControlPainter oldDelegate) {
    return oldDelegate.u != u;
  }
}

class RobotControlPage extends StatefulWidget {
  const RobotControlPage({super.key});

  @override
  State<RobotControlPage> createState() => _RobotControlPageState();
}

class _RobotControlPageState extends State<RobotControlPage> {
  // Networking
  RawDatagramSocket? _socket;
  final int listenPort = 7777;
  String espIp = '192.168.1.47';
  int espPort = 7778;
  bool _udpAvailable = true;

  // Controllers for text fields so they don't get recreated every build
  late TextEditingController _espIpController;
  late TextEditingController _espPortController;

  // Data
  double pitch = 0.0;
  double setPitch = 0.0;
  double controlSignal = 0.0;

  // Rolling buffers for charts
  final int _maxSamples = 500;
  // index counter not currently used; kept for future timestamps if needed
  final List<double> _pitchBuffer = [];
  final List<double> _setPitchBuffer = [];
  final List<double> _uBuffer = [];

  Map<String, double> pidValues = {'P': 2.5, 'I': 900000.0, 'D': 0.0};

  Map<String, int> errorCounts = {
    'READ_TIMEOUT': 0,
    'WRITE_TIMEOUT': 0,
    'READ_ERROR': 0,
    'WRITE_ERROR': 0,
    'SENSOR_READ_FAILED': 0,
  };
  int totalErrors = 0;
  String lastErrorTime = 'None';
  String lastErrorCode = 'None';

  bool saveCsv = true;
  List<List<dynamic>> csvData = [];

  StreamSubscription<RawSocketEvent>? _socketSub;

  @override
  void initState() {
    super.initState();
    _espIpController = TextEditingController(text: espIp);
    _espPortController = TextEditingController(text: espPort.toString());

    if (kIsWeb) {
      // dart:io (RawDatagramSocket / UDP) isn't supported on web builds.
      // Inform the UI and disable UDP functionality.
      _udpAvailable = false;
      debugPrint('UDP (RawDatagramSocket) unavailable on Web; run on a device/emulator.');
    } else {
      _startUdpListener();
    }
  }

  Future<void> _startUdpListener() async {
    try {
      _socket = await RawDatagramSocket.bind(InternetAddress.anyIPv4, listenPort);
      _socket!.readEventsEnabled = true;
      _socketSub = _socket!.listen((event) {
        if (event == RawSocketEvent.read) {
          final datagram = _socket!.receive();
          if (datagram != null) {
            final message = utf8.decode(datagram.data).trim();
            _handleIncoming(message);
          }
        }
      });
      debugPrint('UDP listener started on port $listenPort');
    } catch (e) {
      debugPrint('Failed to start UDP listener: $e');
    }
  }

  void _handleIncoming(String message) {
    if (message.startsWith('I2C_ERROR:')) {
      _parseErrorMessage(message);
      return;
    }
    if (message.startsWith('INIT:')) {
      _parseInitMessage(message);
      return;
    }

    final parts = message.split(',');
    if (parts.length >= 3) {
      setState(() {
        pitch = double.tryParse(parts[0]) ?? pitch;
        setPitch = double.tryParse(parts[1]) ?? setPitch;
        controlSignal = double.tryParse(parts[2]) ?? controlSignal;

        // append to rolling buffers
  _pitchBuffer.add(pitch);
  _setPitchBuffer.add(setPitch);
  _uBuffer.add(controlSignal);
        if (_pitchBuffer.length > _maxSamples) {
          _pitchBuffer.removeAt(0);
          _setPitchBuffer.removeAt(0);
          _uBuffer.removeAt(0);
        }

        if (saveCsv) {
          csvData.add([DateTime.now().toIso8601String(), pitch, setPitch, controlSignal, pidValues['P'], pidValues['I'], pidValues['D']]);
        }
      });
    }
  }

  Widget _buildTelemetryCharts() {
    // Lightweight charts using CustomPaint so we avoid fl_chart SDK issues.
    return Column(
      children: [
        SizedBox(
          height: 160,
          child: Card(
            margin: const EdgeInsets.symmetric(vertical: 4),
            child: Padding(
              padding: const EdgeInsets.all(8.0),
              child: TelemetryChart(
                pitchBuffer: _pitchBuffer,
                setPitchBuffer: _setPitchBuffer,
                title: 'Pitch vs Set Pitch',
              ),
            ),
          ),
        ),
        SizedBox(
          height: 120,
          child: Card(
            margin: const EdgeInsets.symmetric(vertical: 4),
            child: Padding(
              padding: const EdgeInsets.all(8.0),
              child: ControlChart(uBuffer: _uBuffer, title: 'Control Signal (0-255)'),
            ),
          ),
        ),
      ],
    );
  }

  void _parseInitMessage(String message) {
    // Example: INIT:P=2.500,I=0.000001,D=0.000,MP=0.0
    try {
      final parts = message.split(':');
      if (parts.length < 2) return;
      final pidInfo = parts[1];
      final map = <String, double>{};
      for (final pair in pidInfo.split(',')) {
        if (pair.contains('=')) {
          final kv = pair.split('=');
          final k = kv[0].trim();
          final v = double.tryParse(kv[1]) ?? 0.0;
          if (k == 'P' || k == 'I' || k == 'D') map[k] = v;
        }
      }
      setState(() {
        pidValues['P'] = map['P'] ?? pidValues['P']!;
        pidValues['I'] = map['I'] ?? pidValues['I']!;
        pidValues['D'] = map['D'] ?? pidValues['D']!;
      });
    } catch (e) {
      debugPrint('Error parsing INIT: $e');
    }
  }

  void _parseErrorMessage(String message) {
    try {
      // Example: I2C_ERROR:READ_TIMEOUT,code=0x107,total=5
      final parts = message.split(':');
      if (parts.length < 2) return;
      final info = parts[1];
      final items = info.split(',');
      final type = items.isNotEmpty ? items[0] : 'UNKNOWN';
      String code = 'None';
      int total = totalErrors;
      for (final item in items.skip(1)) {
        if (item.contains('=')) {
          final kv = item.split('=');
          if (kv[0] == 'code') code = kv[1];
          if (kv[0] == 'total') total = int.tryParse(kv[1]) ?? total;
        }
      }
      setState(() {
        lastErrorCode = code;
        lastErrorTime = DateTime.now().toIso8601String();
        totalErrors = total;
        if (errorCounts.containsKey(type)) {
          errorCounts[type] = (errorCounts[type] ?? 0) + 1;
        }
      });
    } catch (e) {
      debugPrint('Error parsing I2C_ERROR: $e');
    }
  }

  void _send(String cmd) {
    try {
      if (!_udpAvailable) {
        debugPrint('UDP not available on this platform; cannot send.');
        ScaffoldMessenger.of(context).showSnackBar(const SnackBar(content: Text('UDP unavailable on this platform. Use a device or emulator.')));
        return;
      }

      final data = utf8.encode(cmd);
      // InternetAddress is not supported on web; guarded by _udpAvailable
      final ip = InternetAddress(espIp);
      _socket?.send(data, ip, espPort);
      debugPrint('Sent: $cmd to $espIp:$espPort');
    } catch (e) {
      debugPrint('Send error: $e');
      ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text('Send error: $e')));
    }
  }

  void _sendPidValues() {
    final cmd = 'P=${pidValues['P']},I=${pidValues['I']},D=${pidValues['D']}\\n';
    _send(cmd);
  }

  void _getPidValues() {
    _send('GET\\n');
  }

  void _sendMoveCommand(String direction) {
    _send('MOVE:$direction\\n');
  }

  Future<void> _saveCsvToFile() async {
    try {
      final dir = await getApplicationDocumentsDirectory();
      final fname = 'pid_data_${DateTime.now().toIso8601String().replaceAll(':', '-')}.csv';
      final file = File('${dir.path}/$fname');
      final sink = file.openWrite();
      sink.writeln('Time,Pitch,SetPitch,ControlSignal,P,I,D');
      for (final row in csvData) {
        sink.writeln(row.join(','));
      }
      await sink.flush();
      await sink.close();
      ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text('Saved CSV to ${file.path}')));
    } catch (e) {
      debugPrint('Error saving CSV: $e');
      ScaffoldMessenger.of(context).showSnackBar(const SnackBar(content: Text('Error saving CSV')));
    }
  }

  @override
  void dispose() {
    _socketSub?.cancel();
    _socket?.close();
    super.dispose();
  }

  Widget _buildPidControl(String label, String key, double min, double max, double step) {
    return Column(
      children: [
        Text('$label: ${pidValues[key]!.toStringAsFixed(key == 'I' ? 1 : 3)}'),
        Slider(
          value: pidValues[key]!,
          min: min,
          max: max,
          onChanged: (v) {
            setState(() => pidValues[key] = v);
          },
          onChangeEnd: (v) => _sendPidValues(),
        ),
        SizedBox(
          width: 120,
          child: TextField(
            keyboardType: TextInputType.numberWithOptions(decimal: true),
            controller: TextEditingController(text: pidValues[key]!.toString()),
            onSubmitted: (val) {
              final parsed = double.tryParse(val);
              if (parsed != null) setState(() => pidValues[key] = parsed);
            },
          ),
        )
      ],
    );
  }

  @override
  Widget build(BuildContext context) {
    return DefaultTabController(
      length: 4,
      child: Scaffold(
        appBar: AppBar(
          title: const Text('Robot Control'),
          bottom: const TabBar(tabs: [
            Tab(text: 'Telemetry'),
            Tab(text: 'PID'),
            Tab(text: 'Manual'),
            Tab(text: 'Errors'),
          ]),
        ),
        body: TabBarView(children: [
          // Telemetry Tab
          Padding(
            padding: const EdgeInsets.all(12.0),
            child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
              if (!kIsWeb)
                const SizedBox.shrink()
              else
                Container(
                  padding: const EdgeInsets.all(8),
                  color: Colors.yellow[200],
                  child: const Text('UDP sockets are not available when running in a browser. Run this app on an Android/iOS device or emulator to enable UDP.'),
                ),
              const SizedBox(height: 8),
              Row(
                children: [
                  Expanded(child: Text('Pitch: ${pitch.toStringAsFixed(2)}°', style: const TextStyle(fontSize: 18))),
                  Expanded(child: Text('Set Pitch: ${setPitch.toStringAsFixed(2)}°', style: const TextStyle(fontSize: 18))),
                  Expanded(child: Text('Control: ${controlSignal.toStringAsFixed(1)}', style: const TextStyle(fontSize: 18))),
                ],
              ),
              const SizedBox(height: 12),
              // Charts
              _buildTelemetryCharts(),
              const SizedBox(height: 8),
              Row(children: [
                ElevatedButton(onPressed: saveCsv ? _saveCsvToFile : null, child: const Text('Save CSV Now')),
                const SizedBox(width: 12),
                Row(children: [
                  const Text('Save CSV'),
                  Switch(value: saveCsv, onChanged: (v) => setState(() => saveCsv = v)),
                ])
              ])
            ]),
          ),

          // PID Tab
          Padding(
            padding: const EdgeInsets.all(12.0),
            child: Column(children: [
              Row(
                children: [
                  Expanded(child: _buildPidControl('P Gain', 'P', 0.0, 20.0, 0.01)),
                  Expanded(child: _buildPidControl('I Gain', 'I', 0.0, 900000.0, 1.0)),
                  Expanded(child: _buildPidControl('D Gain', 'D', 0.0, 5.0, 0.01)),
                ],
              ),
              const SizedBox(height: 8),
              Row(children: [
                ElevatedButton(onPressed: _sendPidValues, child: const Text('Send PID Parameters')),
                const SizedBox(width: 8),
                ElevatedButton(onPressed: _getPidValues, child: const Text('Get PID Values')),
              ])
            ]),
          ),

          // Manual Tab
          Center(
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                // Forward
                GestureDetector(
                  onTapDown: (_) => _sendMoveCommand('FORWARD'),
                  onTapUp: (_) => _sendMoveCommand('STOP'),
                  onTapCancel: () => _sendMoveCommand('STOP'),
                  child: Material(
                    color: Theme.of(context).colorScheme.primary,
                    elevation: 4,
                    shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
                    child: SizedBox(
                      width: 140,
                      height: 100,
                      child: const Center(
                        child: Text('↑', style: TextStyle(fontSize: 36, color: Colors.white)),
                      ),
                    ),
                  ),
                ),
                const SizedBox(height: 16),

                // Left / Right row (no STOP in center)
                Row(mainAxisAlignment: MainAxisAlignment.center, children: [
                  GestureDetector(
                    onTapDown: (_) => _sendMoveCommand('LEFT'),
                    onTapUp: (_) => _sendMoveCommand('STOP'),
                    onTapCancel: () => _sendMoveCommand('STOP'),
                    child: Material(
                      color: Theme.of(context).colorScheme.primary,
                      elevation: 4,
                      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
                      child: SizedBox(
                        width: 120,
                        height: 120,
                        child: const Center(child: Text('←', style: TextStyle(fontSize: 34, color: Colors.white))),
                      ),
                    ),
                  ),
                  const SizedBox(width: 24),
                  // spacer where STOP used to be (keeps layout balanced)
                  SizedBox(width: 120, height: 120, child: Container()),
                  const SizedBox(width: 24),
                  GestureDetector(
                    onTapDown: (_) => _sendMoveCommand('RIGHT'),
                    onTapUp: (_) => _sendMoveCommand('STOP'),
                    onTapCancel: () => _sendMoveCommand('STOP'),
                    child: Material(
                      color: Theme.of(context).colorScheme.primary,
                      elevation: 4,
                      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
                      child: SizedBox(
                        width: 120,
                        height: 120,
                        child: const Center(child: Text('→', style: TextStyle(fontSize: 34, color: Colors.white))),
                      ),
                    ),
                  ),
                ]),
                const SizedBox(height: 16),

                // Backward
                GestureDetector(
                  onTapDown: (_) => _sendMoveCommand('BACKWARD'),
                  onTapUp: (_) => _sendMoveCommand('STOP'),
                  onTapCancel: () => _sendMoveCommand('STOP'),
                  child: Material(
                    color: Theme.of(context).colorScheme.primary,
                    elevation: 4,
                    shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
                    child: SizedBox(
                      width: 140,
                      height: 100,
                      child: const Center(
                        child: Text('↓', style: TextStyle(fontSize: 36, color: Colors.white)),
                      ),
                    ),
                  ),
                ),
              ],
            ),
          ),

          // Errors Tab
          Padding(
            padding: const EdgeInsets.all(12.0),
            child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
              const Text('I2C Error Information', style: TextStyle(fontSize: 16, fontWeight: FontWeight.bold)),
              const SizedBox(height: 8),
              Text('Total Errors: $totalErrors'),
              Text('Last Error Time: $lastErrorTime'),
              Text('Last Code: $lastErrorCode'),
              const SizedBox(height: 8),
              Wrap(spacing: 12, children: errorCounts.keys.map((k) => Chip(label: Text('$k: ${errorCounts[k]}'))).toList()),
              const SizedBox(height: 12),
              Row(children: [
                Expanded(
                    child: TextField(
                  decoration: const InputDecoration(labelText: 'ESP IP'),
                  controller: _espIpController,
                  onSubmitted: (v) => setState(() => espIp = v),
                )),
                const SizedBox(width: 8),
                SizedBox(
                  width: 120,
                  child: TextField(
                    decoration: const InputDecoration(labelText: 'ESP Port'),
                    controller: _espPortController,
                    keyboardType: TextInputType.number,
                    onSubmitted: (v) => setState(() => espPort = int.tryParse(v) ?? espPort),
                  ),
                )
              ]),
            ]),
          ),
        ]),
      ),
    );
  }
}
