import 'package:dio/dio.dart';
import 'package:grad_project/core/network/dio_client.dart';
import 'package:grad_project/features/authentication/data/datasources/auth_remote_data_source.dart';
import 'package:grad_project/features/authentication/domain/entities/auth_entity.dart';
import 'package:grad_project/features/authentication/domain/entities/user_entity.dart';
import 'package:grad_project/features/authentication/data/models/user_model.dart';
import 'package:shared_preferences/shared_preferences.dart';

class AuthRemoteDataSourceImpl implements AuthRemoteDataSource {
  final Dio dio = DioClient.dio;

  // -------------------- LOGIN --------------------
  Future<String> signIn(AuthEntity auth) async {
    try {
      final response = await dio.post(
        '/auth/login',
        data: {
          "phone": auth.phone,
          "password": auth.password,
        },
      );

      if (response.statusCode == 200 && response.data['access_token'] != null) {
        final token = response.data['access_token'];
        final prefs = await SharedPreferences.getInstance();
        await prefs.setString('access_token', token);
        return token;
      } else {
        throw Exception('Login failed');
      }
    } on DioError catch (e) {
      throw Exception(e.response?.data ?? 'Login error');
    }
  }

  // -------------------- SIGN UP --------------------
  Future<String> signUp(AuthEntity auth) async {
    try {
      final response = await dio.post(
        '/auth/register',
        data: {
          "phone": auth.phone,
          "password": auth.password,
        },
      );

      if (response.statusCode == 200 && response.data['access_token'] != null) {
        final token = response.data['access_token'];
        final prefs = await SharedPreferences.getInstance();
        await prefs.setString('access_token', token);
        return token;
      } else {
        throw Exception('SignUp failed');
      }
    } on DioError catch (e) {
      throw Exception(e.response?.data ?? 'Signup error');
    }
  }

  // -------------------- LOGOUT --------------------
  Future<void> signOut() async {
    final prefs = await SharedPreferences.getInstance();
    await prefs.remove('access_token');
  }

  // -------------------- GET CURRENT USER --------------------
  Future<UserEntity> getCurrentUser() async {
    final prefs = await SharedPreferences.getInstance();
    final token = prefs.getString('access_token');

    if (token == null) {
      throw Exception('No token found');
    }

    try {
      final response = await dio.get(
        '/me',
        options: Options(
          headers: {'Authorization': 'Bearer $token'},
        ),
      );

      if (response.statusCode == 200) {
        return UserModel.fromJson(response.data);
      } else {
        throw Exception('Failed to fetch current user');
      }
    } on DioError catch (e) {
      throw Exception(e.response?.data ?? 'Get current user error');
    }
  }
}
