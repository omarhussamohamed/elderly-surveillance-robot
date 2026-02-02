import 'package:grad_project/features/authentication/data/datasources/auth_remote_data_source.dart';
import 'package:grad_project/features/authentication/domain/entities/auth_entity.dart';
import 'package:grad_project/features/authentication/domain/entities/user_entity.dart';
import 'package:grad_project/features/authentication/domain/repositories/auth_repo.dart';

class AuthRepoImpl implements AuthRepo {
  final AuthRemoteDataSource authremoteDataSource;

  AuthRepoImpl({required this.authremoteDataSource});

@override
Future<String> signIn(AuthEntity auth) async {
  return await authremoteDataSource.signIn(auth);
}


  @override
  Future<String> signUp(AuthEntity user) async {
    return await authremoteDataSource.signUp(user);
  }

  @override
  Future<void> signOut() async {
    await authremoteDataSource.signOut();
  }

  @override
  Future<UserEntity> getCurrentUser() async {
    return await authremoteDataSource.getCurrentUser();
  }
}
