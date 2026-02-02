
import 'package:elderly_surveillance_mobile/features/home/domain/repositories/home_repo.dart';

class GetTempUseCase {
  final HomeRepo repo;

  GetTempUseCase(this.repo);

  Future<String> call() async {
    return await repo.fetchTemp();
  }
}
