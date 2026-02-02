import 'package:grad_project/features/home/domain/entities/safety_status_entity.dart';

abstract class HomeRepo {
  Future<String> fetchPower();
  Future<String> fetchTemp();
  Future<SafetyStatusEntity> fetchSafetyStatus();
}
