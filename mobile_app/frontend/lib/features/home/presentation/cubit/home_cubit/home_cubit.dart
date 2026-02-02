import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:grad_project/features/home/domain/usecases/get_power_usecase.dart';
import 'package:grad_project/features/home/domain/usecases/get_safety_status_usecase.dart';
import 'package:grad_project/features/home/domain/usecases/get_temp_usecase.dart';

import 'home_state.dart';

class HomeCubit extends Cubit<HomeState> {
  final GetPowerUseCase getPowerUseCase;
  final GetTempUseCase getTempUseCase;
  final GetSafetyStatusUseCase getSafetyStatusUseCase;

  HomeCubit({
    required this.getPowerUseCase,
    required this.getTempUseCase,
    required this.getSafetyStatusUseCase,
  }) : super(HomeInitial());

  /// Load all home data together
  Future<void> loadHomeData() async {
    emit(HomeLoading());

    try {
      final power = await getPowerUseCase();
      final temp = await getTempUseCase();
      final safety = await getSafetyStatusUseCase();

      emit(
        HomeLoaded(
          power: power,
          temperature: temp,
          safetyStatus: safety,
        ),
      );
    } catch (e) {
      emit(HomeError(e.toString()));
    }
  }
}
