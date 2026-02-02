import 'package:dio/dio.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:grad_project/features/authentication/domain/entities/auth_entity.dart';
import 'package:grad_project/features/authentication/domain/entities/user_entity.dart';
import 'package:grad_project/features/authentication/domain/usecases/get_current_user_usecase.dart';
import 'package:grad_project/features/authentication/domain/usecases/sign_in_usecase.dart';
import 'package:grad_project/features/authentication/domain/usecases/sign_out_usecase.dart';
import 'package:grad_project/features/authentication/domain/usecases/sign_up_usecase.dart';
import 'package:grad_project/features/authentication/presentation/cubit/cubit/auth_state.dart';
import 'package:flutter_bloc/flutter_bloc.dart';


class AuthCubit extends Cubit<AuthState> {
  final SignInUsecase signInUseCase;      
  final SignUpUseCase signUpUseCase;
  final SignOutUseCase signOutUseCase;
  final GetCurrentUserUseCase getCurrentUserUseCase;

  AuthCubit({
    required this.signInUseCase,
    required this.signUpUseCase,
    required this.signOutUseCase,
    required this.getCurrentUserUseCase,
  }) : super(AuthInitial());


// ------------------- Login Method ------------------ //
   Future<void> signin({required String phone, required String password}) async {
    emit(AuthLoading());
    try {
      final auth = AuthEntity(phone: phone, password: password); 
      final token = await signInUseCase(auth);
      print('Token from backend: $token');
      emit(AuthSuccess(token: token));
    } on DioError catch (dioError) {
      final message = dioError.response?.data['detail'] ??
                    dioError.response?.statusMessage ??
                    dioError.message;
      emit(AuthError('Login failed: $message'));
    } catch (e) {
      emit(AuthError('Login failed: ${e.toString()}'));
    }
  }


// ------------------- SignUp Method ------------------ //
Future<void> signUp({required String phone, required String password}) async {
  emit(AuthLoading());
  try {
    final auth = AuthEntity(phone: phone, password: password);
    final token = await signUpUseCase(auth); 
    emit(AuthSuccess(token: token));
  } on DioError catch (dioError) {
      final message = dioError.response?.data['detail'] ??
                    dioError.response?.statusMessage ??
                    dioError.message;
      emit(AuthError('Login failed: $message'));
    } catch (e) {
      emit(AuthError('Login failed: ${e.toString()}'));
    }
}


// ------------------- SignOut Method ------------------ //
  Future<void> signOut() async {
    emit(AuthLoading());
    try {
      await signOutUseCase();
      emit(AuthInitial());
    } on DioError catch (dioError) {
      final message = dioError.response?.data['detail'] ??
                    dioError.response?.statusMessage ??
                    dioError.message;
      emit(AuthError('Login failed: $message'));
    } catch (e) {
      emit(AuthError('Login failed: ${e.toString()}'));
    }
  }

// ------------------- Get Current User Method ------------------ //
  Future<void> getCurrentUser() async {
    emit(AuthLoading());
    try {
      final user = await getCurrentUserUseCase();
      emit(AuthSuccess(user: user));
    } on DioError catch (dioError) {
      final message = dioError.response?.data['detail'] ??
                    dioError.response?.statusMessage ??
                    dioError.message;
      emit(AuthError('Login failed: $message'));
    } catch (e) {
      emit(AuthError('Login failed: ${e.toString()}'));
    }
  }
}