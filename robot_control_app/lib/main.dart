import 'dart:async';
import 'dart:convert';
import 'dart:io' show RawDatagramSocket, InternetAddress, File, RawSocketEvent;

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:path_provider/path_provider.dart';
import 'upright_control.dart';

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

// telemetry chartr.
class TelemetryChart extends StatelessWidget {
  final List<double> pitchBuffer;
  final List<double> setPitchBuffer;
  final String title;
  final double? minY;
  final double? maxY;

  const TelemetryChart({Key? key, required this.pitchBuffer, required this.setPitchBuffer, this.title = '', this.minY, this.maxY}) : super(key: key);

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
                painter: _TelemetryPainter(pitch: pitchBuffer, setPitch: setPitchBuffer, minY: minY, maxY: maxY),
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
  final double? minY;
  final double? maxY;

  static const double leftPadding = 44.0;

  _TelemetryPainter({required this.pitch, required this.setPitch, this.minY, this.maxY});

  @override
  void paint(Canvas canvas, Size size) {
  final paintGrid = Paint()..color = Colors.grey.withOpacity(0.25)..strokeWidth = 1.0;
  final paintPitch = Paint()..color = Colors.blue..style = PaintingStyle.stroke..strokeWidth = 2.0;
  final paintSet = Paint()..color = Colors.red..style = PaintingStyle.stroke..strokeWidth = 2.0;

  final plotLeft = leftPadding;
  final plotWidth = size.width - plotLeft;
  final plotHeight = size.height;

    final combined = <double>[];
    combined.addAll(pitch);
    combined.addAll(setPitch);
    double dataMin = combined.isNotEmpty ? combined.reduce((a, b) => a < b ? a : b) : 0.0;
    double dataMax = combined.isNotEmpty ? combined.reduce((a, b) => a > b ? a : b) : 1.0;

    final double finalMinY = (minY ?? dataMin) - 0.0;
    final double finalMaxY = (maxY ?? dataMax) + 0.0;
    double range = finalMaxY - finalMinY;
    if (range.abs() < 1e-6) range = 1.0;

    const int ticks = 4;
    final textStyle = TextStyle(color: Colors.black87, fontSize: 12);
    for (var t = 0; t <= ticks; t++) {
      final dy = t / ticks;
      final y = plotHeight * dy;
      final value = finalMaxY - dy * range;
      canvas.drawLine(Offset(plotLeft, y), Offset(size.width, y), paintGrid);
      final tp = TextPainter(text: TextSpan(text: value.toStringAsFixed(1), style: textStyle), textDirection: TextDirection.ltr);
      tp.layout();
      tp.paint(canvas, Offset(plotLeft - tp.width - 6, y - tp.height / 2));
    }

  final hasAbove = (pitch.isNotEmpty && pitch.last > finalMaxY) || (setPitch.isNotEmpty && setPitch.last > finalMaxY);
  final hasBelow = (pitch.isNotEmpty && pitch.last < finalMinY) || (setPitch.isNotEmpty && setPitch.last < finalMinY);
    final bandPaint = Paint()..color = Colors.red.withOpacity(0.12)..style = PaintingStyle.fill;
    const bandFrac = 0.07;
    if (hasAbove) {
      canvas.drawRect(Rect.fromLTWH(plotLeft, 0, plotWidth, plotHeight * bandFrac), bandPaint);
    }
    if (hasBelow) {
      canvas.drawRect(Rect.fromLTWH(plotLeft, plotHeight * (1 - bandFrac), plotWidth, plotHeight * bandFrac), bandPaint);
    }

    double mapY(double v) => plotHeight - ((v - finalMinY) / range) * plotHeight;

    void drawSeries(List<double> data, Paint normalPaint) {
      if (data.isEmpty) return;
      final n = data.length;

      canvas.save();
      canvas.clipRect(Rect.fromLTWH(plotLeft, 0, plotWidth, plotHeight));

      for (var i = 1; i < n; i++) {
        final x0 = plotLeft + ((i - 1) * plotWidth / (n - 1));
        final x1 = plotLeft + ((i) * plotWidth / (n - 1));
        final v0 = data[i - 1].clamp(finalMinY, finalMaxY);
        final v1 = data[i].clamp(finalMinY, finalMaxY);
        final y0 = mapY(v0);
        final y1 = mapY(v1);
        canvas.drawLine(Offset(x0, y0), Offset(x1, y1), normalPaint);
      }

      if (n == 1) {
        final x = plotLeft + plotWidth / 2;
        final v = data[0].clamp(finalMinY, finalMaxY);
        final y = mapY(v);
        final fill = Paint()..color = normalPaint.color..style = PaintingStyle.fill;
        canvas.drawCircle(Offset(x, y), 2.0, fill);
      }

      canvas.restore();
    }

    drawSeries(setPitch, paintSet);
    drawSeries(pitch, paintPitch);
  }

  @override
  bool shouldRepaint(covariant _TelemetryPainter oldDelegate) {
    return oldDelegate.pitch != pitch || oldDelegate.setPitch != setPitch || oldDelegate.minY != minY || oldDelegate.maxY != maxY;
  }
}

class ControlChart extends StatelessWidget {
  final List<double> uBuffer;
  final String title;
  final double? minY;
  final double? maxY;

  const ControlChart({Key? key, required this.uBuffer, this.title = '', this.minY, this.maxY}) : super(key: key);

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
                painter: _ControlPainter(u: uBuffer, minY: minY, maxY: maxY),
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
  final double? minY;
  final double? maxY;
  static const double leftPadding = 44.0;

  _ControlPainter({required this.u, this.minY, this.maxY});

  @override
  void paint(Canvas canvas, Size size) {
    final paintGrid = Paint()..color = Colors.grey.withOpacity(0.25)..strokeWidth = 1.0;
  final paintU = Paint()..color = Colors.green..style = PaintingStyle.stroke..strokeWidth = 2.0;

    final plotLeft = leftPadding;
    final plotWidth = size.width - plotLeft;
    final plotHeight = size.height;

    const int ticks = 4;
    final double finalMin = minY ?? -255.0;
    final double finalMax = maxY ?? 255.0;
    double range = finalMax - finalMin;
    if (range.abs() < 1e-6) range = 1.0;

    final textStyle = TextStyle(color: Colors.black87, fontSize: 12);
    for (var t = 0; t <= ticks; t++) {
      final dy = t / ticks;
      final y = plotHeight * dy;
      final value = finalMax - dy * range;
      canvas.drawLine(Offset(plotLeft, y), Offset(size.width, y), paintGrid);
      final tp = TextPainter(text: TextSpan(text: value.toStringAsFixed(0), style: textStyle), textDirection: TextDirection.ltr);
      tp.layout();
      tp.paint(canvas, Offset(plotLeft - tp.width - 6, y - tp.height / 2));
    }

  if (u.isEmpty) return;

    final n = u.length;
  final hasAbove = (u.isNotEmpty && u.last > finalMax);
  final hasBelow = (u.isNotEmpty && u.last < finalMin);
    final bandPaint = Paint()..color = Colors.red.withOpacity(0.12)..style = PaintingStyle.fill;
    const bandFrac = 0.07;
    if (hasAbove) {
      canvas.drawRect(Rect.fromLTWH(plotLeft, 0, plotWidth, plotHeight * bandFrac), bandPaint);
    }
    if (hasBelow) {
      canvas.drawRect(Rect.fromLTWH(plotLeft, plotHeight * (1 - bandFrac), plotWidth, plotHeight * bandFrac), bandPaint);
    }

    double mapY(double v) => plotHeight - ((v - finalMin) / range) * plotHeight;

    canvas.save();
    canvas.clipRect(Rect.fromLTWH(plotLeft, 0, plotWidth, plotHeight));
    for (var i = 1; i < n; i++) {
      final x0 = plotLeft + ((i - 1) * plotWidth / (n - 1));
      final x1 = plotLeft + ((i) * plotWidth / (n - 1));
      final v0 = u[i - 1].clamp(finalMin, finalMax);
      final v1 = u[i].clamp(finalMin, finalMax);
      final y0 = mapY(v0);
      final y1 = mapY(v1);
      canvas.drawLine(Offset(x0, y0), Offset(x1, y1), paintU);
    }
    if (n == 1) {
      final x = plotLeft + plotWidth / 2;
      final v = u[0].clamp(finalMin, finalMax);
      final y = mapY(v);
      final fill = Paint()..color = paintU.color..style = PaintingStyle.fill;
      canvas.drawCircle(Offset(x, y), 2.0, fill);
    }
    canvas.restore();
  }

  @override
  bool shouldRepaint(covariant _ControlPainter oldDelegate) {
    return oldDelegate.u != u || oldDelegate.minY != minY || oldDelegate.maxY != maxY;
  }
}

class RobotControlPage extends StatefulWidget {
  const RobotControlPage({super.key});

  @override
  State<RobotControlPage> createState() => _RobotControlPageState();
}

class _RobotControlPageState extends State<RobotControlPage> {
  RawDatagramSocket? _socket;
  final int listenPort = 7777;
  // String espIp = '192.168.1.47';
  String espIp = '192.168.4.1';
  int espPort = 7778;
  bool _udpAvailable = true;

  // Controllers for text fields so they don't get recreated every build
  late TextEditingController _espIpController;
  late TextEditingController _espPortController;

  double pitch = 0.0;
  double setPitch = 0.0;
  double controlSignal = 0.0;

  final int _maxSamples = 500;
  final List<double> _pitchBuffer = [];
  final List<double> _setPitchBuffer = [];
  final List<double> _uBuffer = [];

  Map<String, bool> pidEnabled = {'Kp': true, '1/Ti': true, 'Td': true};
  final Map<String, double> pidOffValues = {'Kp': 0.0, '1/Ti': 0.0, 'Td': 0.0};

  Map<String, double> pidValues = {'Kp': 2.5, '1/Ti': 0.0, 'Td': 0.0, 'UPRIGHT': 177.0};

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
  bool isConnected = false;
  bool _connectDialogShown = false;
  DateTime? _lastReceived;
  Timer? _heartbeatTimer;

  @override
  void initState() {
    super.initState();
    _espIpController = TextEditingController(text: espIp);
    _espPortController = TextEditingController(text: espPort.toString());

    if (kIsWeb) {
      _udpAvailable = false;
      debugPrint('UDP (RawDatagramSocket) unavailable on Web; run on a device/emulator.');
    } else {
      _startUdpListener();
    }

    WidgetsBinding.instance.addPostFrameCallback((_) {
      if (!mounted) return;
      if (!_udpAvailable) return;
      if (!_connectDialogShown && !isConnected) {
        _connectDialogShown = true;
        _showConnectDialog();
      }
    });
  }

  Future<void> _showConnectDialog() async {
    if (!mounted) return;
    await showDialog<void>(
      context: context,
      builder: (context) {
        return AlertDialog(
          title: const Text('Connect to Robot'),
          content: Column(
            mainAxisSize: MainAxisSize.min,
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              Text('ESP IP: $espIp'),
              Text('ESP Port: $espPort'),
              const SizedBox(height: 8),
              const Text('Check if you are connected to the SBR_AP wifi network and click Connect below so the robot can register this device and start sending telemetry.'),
            ],
          ),
          actions: [
            TextButton(onPressed: () => Navigator.of(context).pop(), child: const Text('Cancel')),
            ElevatedButton(
              onPressed: () {
                _sendConnectCommand();
                Navigator.of(context).pop();
              },
              child: const Text('Connect'),
            ),
          ],
        );
      },
    );
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
      _heartbeatTimer?.cancel();
      _heartbeatTimer = Timer.periodic(const Duration(seconds: 1), (_) {

        final now = DateTime.now();
        if (_lastReceived == null) {
          if (!isConnected) return;
          _onDisconnected();
          return;
        }
        final diff = now.difference(_lastReceived!);
        if (diff.inSeconds >= 3) {
          if (isConnected) _onDisconnected();
        } else {
          if (!isConnected) _onConnected();
        }
      });
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

        _lastReceived = DateTime.now();
        if (!isConnected) _onConnected();

  _pitchBuffer.add(pitch);
  _setPitchBuffer.add(setPitch);
  _uBuffer.add(controlSignal);
        if (_pitchBuffer.length > _maxSamples) {
          _pitchBuffer.removeAt(0);
          _setPitchBuffer.removeAt(0);
          _uBuffer.removeAt(0);
        }

        if (saveCsv) {
          csvData.add([DateTime.now().toIso8601String(), pitch, setPitch, controlSignal, pidValues['Kp'], pidValues['Ti'], pidValues['Td'], pidValues['UPRIGHT']]);
        }
      });
    }
  }

  void _onConnected() {
    setState(() {
      isConnected = true;
      _connectDialogShown = false;
    });
    ScaffoldMessenger.of(context).showSnackBar(const SnackBar(content: Text('Connected — receiving data')));
  }

  void _onDisconnected() {
    setState(() {
      isConnected = false;
    });
    if (!_connectDialogShown) {
      _connectDialogShown = true;
      _showConnectDialog();
    }
    ScaffoldMessenger.of(context).showSnackBar(const SnackBar(content: Text('No data — disconnected')));
  }

  Widget _buildTelemetryCharts() {
    final screenH = MediaQuery.of(context).size.height;
    final pitchChartH = screenH * 0.35;
    final controlChartH = screenH * 0.25;

    return Column(
      children: [
        SizedBox(
          height: pitchChartH,
          child: Card(
            margin: const EdgeInsets.symmetric(vertical: 4),
            child: Padding(
              padding: const EdgeInsets.all(8.0),
              child: TelemetryChart(
                pitchBuffer: _pitchBuffer,
                setPitchBuffer: _setPitchBuffer,
                title: 'Pitch vs Set Pitch',
                minY: 150.0,
                maxY: 200.0,
              ),
            ),
          ),
        ),
        SizedBox(
          height: controlChartH,
          child: Card(
            margin: const EdgeInsets.symmetric(vertical: 4),
            child: Padding(
              padding: const EdgeInsets.all(8.0),
              child: ControlChart(uBuffer: _uBuffer, title: 'Control Signal', minY: -255.0, maxY: 255.0),
            ),
          ),
        ),
      ],
    );
  }

  void _parseInitMessage(String message) {
    // Example: INIT:P=2.500,I=0.000001,D=0.000,UPRIGHT=177.000
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
          if (k == 'Kp' || k == '1/Ti' || k == 'Td' || k == 'UPRIGHT') map[k] = v;
        }
      }
      setState(() {
        pidValues['Kp'] = map['Kp'] ?? pidValues['Kp']!;
        pidValues['1/Ti'] = map['1/Ti'] ?? pidValues['1/Ti']!;
        pidValues['Td'] = map['Td'] ?? pidValues['Td']!;
        pidValues['UPRIGHT'] = map['UPRIGHT'] ?? pidValues['UPRIGHT']!;
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
      final ip = InternetAddress(espIp);
      _socket?.send(data, ip, espPort);
      debugPrint('Sent: $cmd to $espIp:$espPort');
    } catch (e) {
      debugPrint('Send error: $e');
      ScaffoldMessenger.of(context).showSnackBar(SnackBar(content: Text('Send error: $e')));
    }
  }

  void _sendPidValues() {
    final cmd = 'Kp=${pidValues['Kp']},1/Ti=${pidValues['1/Ti']},Td=${pidValues['Td']},UPRIGHT=${pidValues['UPRIGHT']}\n';
    _send(cmd);
  }

  void _getPidValues() {
    _send('GET\n');
  }

  void _sendMoveCommand(String direction) {
    _send('MOVE:$direction\n');
  }

  void _sendConnectCommand() {
    _send('CONNECT\n');
    setState(() => isConnected = true);
    ScaffoldMessenger.of(context).showSnackBar(const SnackBar(content: Text('CONNECT sent')));
  }

  Future<void> _saveCsvToFile() async {
    try {
      final dir = await getApplicationDocumentsDirectory();
      final fname = 'pid_data_${DateTime.now().toIso8601String().replaceAll(':', '-')}.csv';
      final file = File('${dir.path}/$fname');
      final sink = file.openWrite();
  sink.writeln('Time,Pitch,SetPitch,ControlSignal,Kp,1/Ti,Td,UPRIGHT');
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
    _heartbeatTimer?.cancel();
    _socketSub?.cancel();
    _socket?.close();
    super.dispose();
  }

  Widget _buildPidControl(String label, String key, double min, double max, double step) {
    final enabled = pidEnabled[key] ?? true;
    final offVal = pidOffValues[key] ?? 0.0;
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 8.0),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Row(
            mainAxisAlignment: MainAxisAlignment.spaceBetween,
            children: [
              Text(
                '$label',
                style: const TextStyle(fontSize: 18, fontWeight: FontWeight.w600),
              ),
              Row(
                children: [
                  Text(enabled ? 'On' : 'Off', style: TextStyle(color: enabled ? Colors.green : Colors.red)),
                  const SizedBox(width: 8),
                  Switch(
                    value: enabled,
                    onChanged: (v) {
                      setState(() {
                        pidEnabled[key] = v;
                        if (!v) {
                          // enforce off value
                          pidValues[key] = offVal;
                        }
                      });
                    },
                  ),
                ],
              ),
            ],
          ),
          const SizedBox(height: 8),
          if (enabled) ...[
            Slider(
              value: pidValues[key]!,
              min: min,
              max: max,
              onChanged: (v) {
                final snapped = (v / step).round() * step;
                setState(() => pidValues[key] = snapped.clamp(min, max));
              },
            ),
            const SizedBox(height: 6),
            TextField(
              keyboardType: TextInputType.numberWithOptions(decimal: true),
              controller: TextEditingController(text: pidValues[key]!.toString()),
              decoration: const InputDecoration(
                border: OutlineInputBorder(),
                isDense: true,
                contentPadding: EdgeInsets.symmetric(vertical: 12, horizontal: 8),
              ),
              style: const TextStyle(fontSize: 16),
              onSubmitted: (val) {
                final parsed = double.tryParse(val);
                if (parsed != null) setState(() => pidValues[key] = parsed.clamp(min, max));
              },
            ),
          ] else ...[
            Padding(
              padding: const EdgeInsets.only(top: 6.0, bottom: 6.0),
              child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
                Text('Disabled — value enforced to ${offVal.toString()}'),
              ]),
            )
          ]
        ],
      ),
    );
  }

  @override
  Widget build(BuildContext context) {
    return DefaultTabController(
      length: 4,
      child: Scaffold(
        appBar: AppBar(
          title: Row(
            children: [
              const Text('Robot Control'),
              const SizedBox(width: 8),
              Container(
                width: 10,
                height: 10,
                decoration: BoxDecoration(
                  color: isConnected ? Colors.green : Colors.red,
                  shape: BoxShape.circle,
                ),
              ),
              const SizedBox(width: 6),
              Text(isConnected ? 'Connected' : 'Disconnected', style: const TextStyle(fontSize: 12)),
            ],
          ),
          bottom: const TabBar(tabs: [
        Tab(text: 'Telemetry'),
      Tab(text: 'Parameters'),
      Tab(text: 'Manual'),
            Tab(text: 'Config & Errors'),
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
                  Expanded(
                      child: Text('Pitch: ${pitch.toStringAsFixed(2)}°',
                          style: TextStyle(fontSize: 18, color: Colors.blue))),
                  Expanded(
                      child: Text('Set Pitch: ${setPitch.toStringAsFixed(2)}°',
                          style: TextStyle(fontSize: 18, color: Colors.red))),
                  Expanded(
                      child: Text('Control: ${controlSignal.toStringAsFixed(1)}',
                          style: TextStyle(fontSize: 18, color: Colors.green))),
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

          // PID Tab (vertical layout for touch)
          Padding(
            padding: const EdgeInsets.all(16.0),
            child: Column(
              children: [
                // controls scrollable area
                Expanded(
                  child: SingleChildScrollView(
                    child: Column(
                      crossAxisAlignment: CrossAxisAlignment.stretch,
                      children: [
                        _buildPidControl('Kp Constant', 'Kp', 0.0, 20.0, 0.1),
                        const SizedBox(height: 12),
                        _buildPidControl('1/Ti Constant', '1/Ti', 0.0, 2.0, 0.01),
                        const SizedBox(height: 12),
                        _buildPidControl('Td Constant', 'Td', 0.0, 2.0, 0.01),
                        const SizedBox(height: 12),
                        UprightControl(
                          label: 'Upright Pitch',
                          value: pidValues['UPRIGHT']!,
                          min: 150.0,
                          max: 200.0,
                          onChanged: (v) {
                            final snapped = (v * 10).round() / 10.0;
                            setState(() => pidValues['UPRIGHT'] = snapped.clamp(150.0, 200.0));
                          },
                          onSubmitted: (v) => setState(() => pidValues['UPRIGHT'] = v.clamp(150.0, 200.0)),
                        ),
                      ],
                    ),
                  ),
                ),

                const SizedBox(height: 12),
                      Wrap(
                        alignment: WrapAlignment.end,
                        spacing: 12,
                        children: [
                          ElevatedButton(onPressed: _sendPidValues, child: const Text('Send Params')),
                          ElevatedButton(onPressed: _getPidValues, child: const Text('Get Params')),
                        ],
                      )
              ],
            ),
          ),

          // Manual Tab
          Padding(
            padding: const EdgeInsets.all(12.0),
            child: LayoutBuilder(
              builder: (context, constraints) {
                final w = constraints.maxWidth;
                final double big = w > 420 ? 160 : (w * 0.35).clamp(80, 160);
                final double mid = w > 420 ? 140 : (w * 0.35).clamp(64, 140);

                Widget buildButton(String label, double size, VoidCallback onDown, VoidCallback onUp) {
                  IconData? icon;
                  switch (label) {
                    case '↑':
                      icon = Icons.arrow_upward;
                      break;
                    case '↓':
                      icon = Icons.arrow_downward;
                      break;
                    case '←':
                      icon = Icons.arrow_back;
                      break;
                    case '→':
                      icon = Icons.arrow_forward;
                      break;
                    default:
                      icon = null;
                  }

                  return Material(
                    color: Theme.of(context).colorScheme.primary,
                    elevation: 4,
                    shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(12)),
                    child: InkWell(
                      borderRadius: BorderRadius.circular(12),
                      onTap: () {},
                      onTapDown: (_) => onDown(),
                      onTapUp: (_) => onUp(),
                      onTapCancel: () => onUp(),
                      child: SizedBox(
                        width: size,
                        height: size,
                        child: Center(
                          child: icon != null
                              ? Icon(icon, size: size * 0.45, color: Colors.white)
                              : Text(label, style: TextStyle(fontSize: size * 0.28, color: Colors.white)),
                        ),
                      ),
                    ),
                  );
                }

                return Column(
                  mainAxisAlignment: MainAxisAlignment.center,
                  children: [
                    Center(child: buildButton('↑', big, () => _sendMoveCommand('FORWARD'), () => _sendMoveCommand('STOP'))),
                    SizedBox(height: 16),

                    Row(
                      children: [
                        Expanded(child: Center(child: buildButton('←', mid, () => _sendMoveCommand('LEFT'), () => _sendMoveCommand('STOP')))),
                        const SizedBox(width: 12),
                        Expanded(child: Center(child: buildButton('→', mid, () => _sendMoveCommand('RIGHT'), () => _sendMoveCommand('STOP')))),
                      ],
                    ),

                    const SizedBox(height: 16),
                    Center(child: buildButton('↓', big, () => _sendMoveCommand('BACKWARD'), () => _sendMoveCommand('STOP'))),
                  ],
                );
              },
            ),
          ),

          Padding(
            padding: const EdgeInsets.all(12.0),
            child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
              const Text('Config & Errors', style: TextStyle(fontSize: 16, fontWeight: FontWeight.bold)),
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
              const SizedBox(height: 12),
              Row(
                children: [
                  ElevatedButton(
                    onPressed: _sendConnectCommand,
                    child: const Text('Connect to Robot'),
                  ),
                ],
              ),
            ]),
          ),
        ]),
      ),
    );
  }
}
