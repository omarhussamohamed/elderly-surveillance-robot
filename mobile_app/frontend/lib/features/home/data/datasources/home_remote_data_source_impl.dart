import 'package:dio/dio.dart';
import 'package:grad_project/features/home/domain/entities/safety_status_entity.dart';
import 'home_remote_data_source.dart';

class HomeRemoteDataSourceImpl implements HomeRemoteDataSource {
  final Dio dio;

  HomeRemoteDataSourceImpl({required this.dio});

  static const _powerEndpoint = '/system/power';
  static const _tempEndpoint = '/system/temp';
  static const _safetyEndpoint = '/system/safety';

  @override
  Future<String> getPower() async {
    try {
      final response = await dio.get(_powerEndpoint);
      return response.data['power'] ?? '0%';
    } catch (e) {
      throw Exception('Failed to load power');
    }
  }

  @override
  Future<String> getTemp() async {
    try {
      final response = await dio.get(_tempEndpoint);
      return response.data['temp'] ?? '0°C';
    } catch (e) {
      throw Exception('Failed to load temperature');
    }
  }

  @override
  Future<SafetyStatusEntity> getSafetyStatus() async {
    try {
      final response = await dio.get(_safetyEndpoint);
      final data = response.data;

      return SafetyStatusEntity(
        gas: data['gas'] ?? 'Unknown',
        fire: data['fire'] ?? 'Unknown',
        fall: data['fall'] ?? 'Unknown',
        stranger: data['stranger'] ?? 'Unknown',
      );
    } catch (e) {
      throw Exception('Failed to load safety status');
    }
  }
}
