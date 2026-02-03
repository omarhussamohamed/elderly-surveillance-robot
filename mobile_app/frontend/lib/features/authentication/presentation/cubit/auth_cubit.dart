import 'package:dio/dio.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:elderly_surveillance_mobile/features/authentication/domain/entities/auth_entity.dart';
import 'package:elderly_surveillance_mobile/features/authentication/domain/entities/user_entity.dart';
import 'package:elderly_surveillance_mobile/features/authentication/domain/usecases/get_current_user_usecase.dart';
import 'package:elderly_surveillance_mobile/features/authentication/domain/usecases/sign_in_usecase.dart';
import 'package:elderly_surveillance_mobile/features/authentication/domain/usecases/sign_out_usecase.dart';
import 'package:elderly_surveillance_mobile/features/authentication/domain/usecases/sign_up_usecase.dart';
import 'package:elderly_surveillance_mobile/features/authentication/presentation/cubit/auth_state.dart';


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
      emit(AuthSuccess(token: token));
    } on DioException catch (dioError) {
      final message = _extractErrorMessage(dioError);
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
  } on DioException catch (dioError) {
      final message = _extractErrorMessage(dioError);
      emit(AuthError('Sign up failed: $message'));
    } catch (e) {
      emit(AuthError('Sign up failed: ${e.toString()}'));
    }
}


// ------------------- SignOut Method ------------------ //
  Future<void> signOut() async {
    emit(AuthLoading());
    try {
      await signOutUseCase();
      emit(AuthInitial());
    } on DioException catch (dioError) {
      final message = _extractErrorMessage(dioError);
      emit(AuthError('Sign out failed: $message'));
    } catch (e) {
      emit(AuthError('Sign out failed: ${e.toString()}'));
    }
  }

// ------------------- Get Current User Method ------------------ //
  Future<void> getCurrentUser() async {
    emit(AuthLoading());
    try {
      final user = await getCurrentUserUseCase();
      emit(AuthSuccess(user: user));
    } on DioException catch (dioError) {
      final message = _extractErrorMessage(dioError);
      emit(AuthError('Failed to get user: $message'));
    } catch (e) {
      emit(AuthError('Failed to get user: ${e.toString()}'));
    }
  }

  /// Extract user-friendly error message from DioException
  String _extractErrorMessage(DioException error) {
    // Handle network timeouts
    if (error.type == DioExceptionType.connectionTimeout ||
        error.type == DioExceptionType.receiveTimeout ||
        error.type == DioExceptionType.sendTimeout) {
      return 'Connection timeout. Please check your network.';
    }
    // Handle no connection
    if (error.type == DioExceptionType.connectionError) {
      return 'No internet connection.';
    }
    // Handle server response errors
    return error.response?.data['detail'] ??
           error.response?.statusMessage ??
           error.message ??
           'Unknown error occurred';
  }
}