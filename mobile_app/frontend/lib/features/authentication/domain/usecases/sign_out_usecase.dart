import 'package:elderly_surveillance_mobile/features/authentication/domain/repositories/auth_repo.dart';

class SignOutUseCase {
  final AuthRepo authRepo;

  SignOutUseCase(this.authRepo);

  Future<void> call() async {
    await authRepo.signOut();
  }
}
