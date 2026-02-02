

import 'package:elderly_surveillance_mobile/features/home/domain/entities/safety_status_entity.dart';
import 'package:elderly_surveillance_mobile/features/home/domain/repositories/home_repo.dart';

class GetSafetyStatusUseCase {
  final HomeRepo repo;

  GetSafetyStatusUseCase(this.repo);

  Future<SafetyStatusEntity> call() async {
    return await repo.fetchSafetyStatus();
  }
}
