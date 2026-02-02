import 'package:equatable/equatable.dart';
import 'package:grad_project/features/home/domain/entities/safety_status_entity.dart';

abstract class HomeState extends Equatable {
  const HomeState();

  @override
  List<Object?> get props => [];
}

class HomeInitial extends HomeState {}

class HomeLoading extends HomeState {}

class HomeLoaded extends HomeState {
  final String power;
  final String temperature;
  final SafetyStatusEntity safetyStatus;

  const HomeLoaded({
    required this.power,
    required this.temperature,
    required this.safetyStatus,
  });

  @override
  List<Object?> get props => [power, temperature, safetyStatus];
}

class HomeError extends HomeState {
  final String message;

  const HomeError(this.message);

  @override
  List<Object?> get props => [message];
}
