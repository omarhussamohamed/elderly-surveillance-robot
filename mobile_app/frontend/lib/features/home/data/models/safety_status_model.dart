class SafetyStatus {
  final String gas;
  final String fire;
  final String fall;
  final String stranger;

  SafetyStatus({
    required this.gas,
    required this.fire,
    required this.fall,
    required this.stranger,
  });

  factory SafetyStatus.fromJson(Map<String, dynamic> json) {
    return SafetyStatus(
      gas: json['gas'],
      fire: json['fire'],
      fall: json['fall'],
      stranger: json['stranger'],
    );
  }
}
