
import 'package:grad_project/features/home/domain/entities/safety_status_entity.dart';

abstract class HomeRemoteDataSource {
  Future<String> getPower();
  Future<String> getTemp();
  Future<SafetyStatusEntity> getSafetyStatus();
}