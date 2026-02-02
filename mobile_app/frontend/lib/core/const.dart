import 'package:flutter/material.dart';

class AppSizes {
  static double screenWidth(BuildContext context) {
    return MediaQuery.of(context).size.width;
  }

  static double screenHeight(BuildContext context) {
    return MediaQuery.of(context).size.height;
  }
}

class AppColors {
  static const Color darkNavy = Color.fromARGB(16, 25, 34, 100);
  static const Color lightNavy = Color.fromARGB(15, 23, 42, 100);
  static const Color white = Color.fromARGB(255, 255, 255, 255);
  static const Color grey = Color.fromARGB(82, 96, 118, 100);
  static const Color green = Color.fromARGB(255, 35, 195, 93);
  static const Color blue = Color.fromARGB(255, 19, 124, 229);
  static const Color paleBlue = Color.fromARGB(50, 19, 124, 229);
  static const Color paleGreen = Color.fromARGB(50, 34, 195, 93);
  static const Color red = Color.fromARGB(255, 204, 34, 0);
}

class ScreenConst {
  static const String homeScreen = 'homeScreen';
  static const String mainScreen = 'mainScreen';
  static const String signInSignUpScreen = 'signInSignUpScreen';
  static const String emergencyCallScreen = 'emergencyCallScreen';
  static const String managePeopleScreen = 'managePeopleScreen';
  static const String profileScreen = 'profileScreen';
  static const String HelpSupportScreen = 'HelpSupportScreen';
  static const String notificationScreen = 'notificationScreen';
}
