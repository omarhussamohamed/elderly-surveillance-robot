import 'package:elderly_surveillance_mobile/features/home/domain/entities/safety_status_entity.dart';

abstract class HomeRepo {
  Future<String> fetchPower();
  Future<String> fetchTemp();
  Future<SafetyStatusEntity> fetchSafetyStatus();
}
