class SafetyStatusEntity {
  final String gas;
  final String fire;
  final String fall;
  final String stranger;

  SafetyStatusEntity({
    required this.gas,
    required this.fire,
    required this.fall,
    required this.stranger,
  });
}

class HomeStatusEntity {
  final String power;
  final String temp;
  final SafetyStatusEntity safety;

  HomeStatusEntity({
    required this.power,
    required this.temp,
    required this.safety,
  });
}
