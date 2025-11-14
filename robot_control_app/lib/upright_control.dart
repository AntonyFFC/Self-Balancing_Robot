import 'package:flutter/material.dart';

typedef DoubleCallback = void Function(double);

class UprightControl extends StatelessWidget {
  final String label;
  final double value;
  final double min;
  final double max;
  final ValueChanged<double> onChanged;
  final ValueChanged<double> onSubmitted;

  const UprightControl({super.key, required this.label, required this.value, required this.min, required this.max, required this.onChanged, required this.onSubmitted});

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 8.0),
      child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
        Text(label, style: const TextStyle(fontSize: 18, fontWeight: FontWeight.w600)),
        const SizedBox(height: 8),
        Slider(value: value, min: min, max: max, onChanged: onChanged),
        const SizedBox(height: 6),
        TextField(
          keyboardType: TextInputType.numberWithOptions(decimal: true),
          controller: TextEditingController(text: value.toString()),
          decoration: const InputDecoration(border: OutlineInputBorder(), isDense: true, contentPadding: EdgeInsets.symmetric(vertical: 12, horizontal: 8)),
          onSubmitted: (s) {
            final parsed = double.tryParse(s);
            if (parsed != null) onSubmitted(parsed);
          },
        )
      ]),
    );
  }
}
