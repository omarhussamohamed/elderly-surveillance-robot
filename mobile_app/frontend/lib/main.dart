import 'package:flutter/material.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:grad_project/core/on_generate_route.dart';
import 'package:grad_project/features/authentication/data/datasources/auth_remote_data_source_impl.dart';
import 'package:grad_project/features/authentication/presentation/screens/sign_in_sign_up_screen.dart';
import 'package:grad_project/core/const.dart';

// Cubit & Usecases
import 'package:grad_project/features/authentication/presentation/cubit/cubit/auth_cubit.dart';
import 'package:grad_project/features/authentication/presentation/cubit/cubit/auth_state.dart';
import 'package:grad_project/features/authentication/domain/usecases/sign_in_usecase.dart';
import 'package:grad_project/features/authentication/domain/usecases/sign_up_usecase.dart';
import 'package:grad_project/features/authentication/domain/usecases/sign_out_usecase.dart';
import 'package:grad_project/features/authentication/domain/usecases/get_current_user_usecase.dart';

// Repo & DataSource
import 'package:grad_project/features/authentication/data/repositories/auth_repo_impl.dart';
import 'package:grad_project/features/authentication/data/datasources/auth_remote_data_source.dart';
import 'package:grad_project/welcome_screen.dart';

void main() {
  runApp(const MyApp());
}

class MyApp extends StatelessWidget {
  const MyApp({super.key});

  @override
  Widget build(BuildContext context) {
    // إنشاء الـ repository و data source
    final authRepo = AuthRepoImpl(
      authremoteDataSource: AuthRemoteDataSourceImpl(),
    );

    return BlocProvider<AuthCubit>(
      create:
          (context) => AuthCubit(
            signInUseCase: SignInUsecase(authRepo),
            signUpUseCase: SignUpUseCase(authRepo),
            signOutUseCase: SignOutUseCase(authRepo),
            getCurrentUserUseCase: GetCurrentUserUseCase(authRepo),
          ),
      child: MaterialApp(
        debugShowCheckedModeBanner: false,
        theme: ThemeData(
          scaffoldBackgroundColor: AppColors.darkNavy,
          appBarTheme: AppBarTheme(
            backgroundColor: AppColors.lightNavy,
            titleTextStyle: TextStyle(color: AppColors.white),
          ),
          bottomNavigationBarTheme: BottomNavigationBarThemeData(
            backgroundColor: AppColors.lightNavy,
            selectedItemColor: AppColors.blue,
            selectedLabelStyle: TextStyle(
              fontFamily: 'Montserrat',
              fontSize: 11,
            ),
            unselectedLabelStyle: TextStyle(
              fontFamily: 'Montserrat',
              fontSize: 11,
            ),
            unselectedItemColor: AppColors.blue,
            showUnselectedLabels: true,
          ),
        ),
        onGenerateRoute: OnGenerateRoute.route,

        home: WelcomeScreen(),
      ),
    );
  }
}
