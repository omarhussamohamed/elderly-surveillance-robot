

import 'package:grad_project/features/home/domain/entities/safety_status_entity.dart';
import 'package:grad_project/features/home/domain/repositories/home_repo.dart';

class GetSafetyStatusUseCase {
  final HomeRepo repo;

  GetSafetyStatusUseCase(this.repo);

  Future<SafetyStatusEntity> call() async {
    return await repo.fetchSafetyStatus();
  }
}
