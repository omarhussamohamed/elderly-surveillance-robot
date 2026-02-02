import 'package:elderly_surveillance_mobile/features/authentication/domain/entities/auth_entity.dart';
import 'package:elderly_surveillance_mobile/features/authentication/domain/entities/user_entity.dart';
import 'package:elderly_surveillance_mobile/features/authentication/domain/repositories/auth_repo.dart';

class SignUpUseCase {
  final AuthRepo authRepo;

  SignUpUseCase(this.authRepo);

  Future<String> call(AuthEntity user) async {
    return await authRepo.signUp(user);
  }
}
