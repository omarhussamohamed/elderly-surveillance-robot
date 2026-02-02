
import 'package:grad_project/features/home/domain/repositories/home_repo.dart';

class GetTempUseCase {
  final HomeRepo repo;

  GetTempUseCase(this.repo);

  Future<String> call() async {
    return await repo.fetchTemp();
  }
}
