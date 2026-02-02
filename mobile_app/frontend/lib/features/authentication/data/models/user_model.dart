import 'package:elderly_surveillance_mobile/features/authentication/domain/entities/user_entity.dart';

class UserModel extends UserEntity {
  UserModel({required super.id, required super.phone});

  factory UserModel.fromJson(Map<String, dynamic> json) {
    return UserModel(
      id: json['id'],
      phone: json['phone'],
    );
  }
}
