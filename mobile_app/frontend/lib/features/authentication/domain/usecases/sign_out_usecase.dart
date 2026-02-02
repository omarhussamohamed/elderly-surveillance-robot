import 'package:grad_project/features/authentication/domain/repositories/auth_repo.dart';

class SignOutUseCase {
  final AuthRepo authRepo;

  SignOutUseCase(this.authRepo);

  Future<void> call() async {
    await authRepo.signOut();
  }
}
