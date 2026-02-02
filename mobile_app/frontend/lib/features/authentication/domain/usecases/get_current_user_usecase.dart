import 'package:elderly_surveillance_mobile/features/authentication/domain/entities/user_entity.dart';
import 'package:elderly_surveillance_mobile/features/authentication/domain/repositories/auth_repo.dart';

class GetCurrentUserUseCase {
  final AuthRepo authRepo;

  GetCurrentUserUseCase(this.authRepo);

  Future<UserEntity> call() async {
    return await authRepo.getCurrentUser();
  }
}
