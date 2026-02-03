import 'package:dio/dio.dart';
import 'package:elderly_surveillance_mobile/features/home/domain/entities/safety_status_entity.dart';
import 'home_remote_data_source.dart';

/// Error messages for network operations
class NetworkErrors {
  static const timeout = 'Connection timeout. Please check your network.';
  static const malformedResponse = 'Invalid response from server.';
  static const connectionError = 'Unable to connect to server.';
}

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
      if (response.data is! Map<String, dynamic>) {
        throw Exception(NetworkErrors.malformedResponse);
      }
      return response.data['power']?.toString() ?? '0%';
    } on DioException catch (e) {
      throw Exception(_mapDioError(e, 'Failed to load power'));
    }
  }

  @override
  Future<String> getTemp() async {
    try {
      final response = await dio.get(_tempEndpoint);
      if (response.data is! Map<String, dynamic>) {
        throw Exception(NetworkErrors.malformedResponse);
      }
      return response.data['temp']?.toString() ?? '0°C';
    } on DioException catch (e) {
      throw Exception(_mapDioError(e, 'Failed to load temperature'));
    }
  }

  @override
  Future<SafetyStatusEntity> getSafetyStatus() async {
    try {
      final response = await dio.get(_safetyEndpoint);
      if (response.data is! Map<String, dynamic>) {
        throw Exception(NetworkErrors.malformedResponse);
      }
      final data = response.data as Map<String, dynamic>;

      return SafetyStatusEntity(
        gas: data['gas']?.toString() ?? 'Unknown',
        fire: data['fire']?.toString() ?? 'Unknown',
        fall: data['fall']?.toString() ?? 'Unknown',
        stranger: data['stranger']?.toString() ?? 'Unknown',
      );
    } on DioException catch (e) {
      throw Exception(_mapDioError(e, 'Failed to load safety status'));
    }
  }

  /// Map DioException to user-friendly error message
  String _mapDioError(DioException e, String fallback) {
    switch (e.type) {
      case DioExceptionType.connectionTimeout:
      case DioExceptionType.receiveTimeout:
      case DioExceptionType.sendTimeout:
        return NetworkErrors.timeout;
      case DioExceptionType.connectionError:
        return NetworkErrors.connectionError;
      default:
        return e.response?.data?['detail']?.toString() ?? fallback;
    }
  }
}
