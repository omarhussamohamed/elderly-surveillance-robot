import 'package:flutter/material.dart';
import 'package:grad_project/core/const.dart';
import 'package:grad_project/features/emergency/presentation/screens/emergency_screen.dart';
import 'package:grad_project/features/authentication/presentation/screens/sign_in_sign_up_screen.dart';
import 'package:grad_project/features/home/presentation/screens/home_screen.dart';
import 'package:grad_project/features/profile/presentation/screens/help_support_screen.dart';
import 'package:grad_project/features/main_screen/main_screen.dart';
import 'package:grad_project/features/profile/presentation/screens/manage_people_screen.dart';
import 'package:grad_project/features/notifications/notification.dart';
import 'package:grad_project/features/profile/presentation/screens/profile_screen.dart';

class OnGenerateRoute {
  //User? user = FirebaseAuth.instance.currentUser;

  static Route<dynamic>? route(RouteSettings settings) {
    switch (settings.name) {
      case ScreenConst.homeScreen:
        {
          return _fadeRoute(HomeScreen());
        }
      case ScreenConst.mainScreen:
        {
          return _fadeRoute(MainScreen());
        }
      case ScreenConst.signInSignUpScreen:
        {
          return _fadeRoute(SignInSignUpScreen());
        }
      case ScreenConst.emergencyCallScreen:
        {
          return _fadeRoute(EmergencyCall());
        }
        case ScreenConst.managePeopleScreen:
        {
          return _fadeRoute(ManagePeopleScreen());
        }
          case ScreenConst.profileScreen:
        {
          return _fadeRoute(ProfileScreen());
        }
  case ScreenConst.helpSupportScreen:
        {
          return _fadeRoute(HelpSupportScreen());
        }
         case ScreenConst.notificationScreen:
        {
          return _fadeRoute(NotificationScreen());
        }


      default:
        return MaterialPageRoute(builder: (_) => const NoScreenFound());
    }
  }

  static PageRouteBuilder _fadeRoute(Widget page) {
    return PageRouteBuilder(
      pageBuilder: (context, animation, secondaryAnimation) => page,
      transitionsBuilder: (context, animation, secondaryAnimation, child) {
        return FadeTransition(opacity: animation, child: child);
      },
    );
  }
}

class NoScreenFound extends StatelessWidget {
  const NoScreenFound({super.key});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: Text('Page not found')),
      body: Center(child: Text('Page not found')),
    );
  }
}
