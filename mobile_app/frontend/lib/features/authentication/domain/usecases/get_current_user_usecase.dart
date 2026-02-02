import 'package:grad_project/features/authentication/domain/entities/user_entity.dart';
import 'package:grad_project/features/authentication/domain/repositories/auth_repo.dart';

class GetCurrentUserUseCase {
  final AuthRepo authRepo;

  GetCurrentUserUseCase(this.authRepo);

  Future<UserEntity> call() async {
    return await authRepo.getCurrentUser();
  }
}
