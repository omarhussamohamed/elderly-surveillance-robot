// auth_state.dart
import 'package:equatable/equatable.dart';
import 'package:grad_project/features/authentication/domain/entities/user_entity.dart';

abstract class AuthState extends Equatable {
  const AuthState();

  @override
  List<Object?> get props => [];
}

class AuthInitial extends AuthState {}


class AuthLoading extends AuthState {}


class AuthSuccess extends AuthState {
  final String? token;      
  final UserEntity? user;   
  const AuthSuccess({this.token, this.user});

  @override
  List<Object?> get props => [token, user];
}


class AuthError extends AuthState {
  final String message;

  const AuthError(this.message);

  @override
  List<Object?> get props => [message];
}
