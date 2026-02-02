import 'package:elderly_surveillance_mobile/features/authentication/domain/entities/auth_entity.dart';
import 'package:elderly_surveillance_mobile/features/authentication/domain/entities/user_entity.dart';
import 'package:elderly_surveillance_mobile/features/authentication/domain/repositories/auth_repo.dart';

class SignInUsecase {
  final AuthRepo authRepo;

  SignInUsecase(this.authRepo);

  Future<String> call(AuthEntity auth) async {
    return await authRepo.signIn(auth);
  }
}
