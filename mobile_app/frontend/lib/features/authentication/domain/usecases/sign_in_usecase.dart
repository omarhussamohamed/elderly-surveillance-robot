import 'package:grad_project/features/authentication/domain/entities/auth_entity.dart';
import 'package:grad_project/features/authentication/domain/entities/user_entity.dart';
import 'package:grad_project/features/authentication/domain/repositories/auth_repo.dart';

class SignInUsecase {
  final AuthRepo authRepo;

  SignInUsecase(this.authRepo);

  Future<String> call(AuthEntity auth) async {
    return await authRepo.signIn(auth);
  }
}
