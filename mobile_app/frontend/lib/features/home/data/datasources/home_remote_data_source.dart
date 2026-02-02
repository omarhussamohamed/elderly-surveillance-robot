
import 'package:elderly_surveillance_mobile/features/home/domain/entities/safety_status_entity.dart';

abstract class HomeRemoteDataSource {
  Future<String> getPower();
  Future<String> getTemp();
  Future<SafetyStatusEntity> getSafetyStatus();
}