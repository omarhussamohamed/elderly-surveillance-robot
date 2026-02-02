import 'package:grad_project/features/home/data/datasources/home_remote_data_source.dart';
import 'package:grad_project/features/home/domain/entities/safety_status_entity.dart';
import 'package:grad_project/features/home/domain/repositories/home_repo.dart';


class HomeRepoImpl implements HomeRepo {
  final HomeRemoteDataSource remoteDataSource;

  HomeRepoImpl({required this.remoteDataSource});

  @override
  Future<String> fetchPower() async {
    return await remoteDataSource.getPower();
  }

  @override
  Future<String> fetchTemp() async {
    return await remoteDataSource.getTemp();
  }

  @override
  Future<SafetyStatusEntity> fetchSafetyStatus() async {
    return await remoteDataSource.getSafetyStatus();
  }
}
