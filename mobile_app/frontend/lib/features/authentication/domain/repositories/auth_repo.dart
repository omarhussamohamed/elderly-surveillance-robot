import 'package:grad_project/features/authentication/domain/entities/auth_entity.dart';
import 'package:grad_project/features/authentication/domain/entities/user_entity.dart';

abstract class AuthRepo {
  Future<String> signIn(AuthEntity auth);

  Future<String> signUp(AuthEntity user);

  Future<void> signOut();

  Future<UserEntity> getCurrentUser();
}
