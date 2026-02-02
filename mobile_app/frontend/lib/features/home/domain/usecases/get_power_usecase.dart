import 'package:elderly_surveillance_mobile/features/home/domain/repositories/home_repo.dart';

class GetPowerUseCase {
  final HomeRepo repo;

  GetPowerUseCase(this.repo);

  Future<String> call() async {
    return await repo.fetchPower();
  }
}
