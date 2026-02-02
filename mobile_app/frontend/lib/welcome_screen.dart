import 'package:flutter/material.dart';
import 'package:flutter_svg/flutter_svg.dart';
import 'package:grad_project/core/const.dart';

class WelcomeScreen extends StatelessWidget {
  const WelcomeScreen({super.key});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Center(
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            // Text(
            //   'LINK',
            //   style: TextStyle(
            //     color: AppColors.blue,
            //     fontSize: 24,
            //     fontFamily: 'Montserrat',
            //   ),
            // ),
            SvgPicture.asset(
              'assets/images/link_logo/link_logo.svg',
              width: 97,
              height: 120,
            ),
            // Image.asset('assets/link_logo.png', width: 250, height: 250),
            SizedBox(height: 50),
            ElevatedButton(
              style: ElevatedButton.styleFrom(
                backgroundColor: AppColors.blue,
                fixedSize: Size(322, 37),
                side: BorderSide(color: AppColors.blue, width: 0.66),
                shape: RoundedRectangleBorder(
                  borderRadius: BorderRadius.circular(0),
                ),
              ),
              onPressed: () {
                Navigator.pushNamed(context, ScreenConst.signInSignUpScreen);
              },
              child: Text(
                'SIGN IN',
                style: TextStyle(
                  color: AppColors.white,
                  fontSize: 15,
                  fontFamily: 'Montserrat',
                ),
                textAlign: TextAlign.center,
              ),
            ),

            SizedBox(height: 9),
            ElevatedButton(
              style: ElevatedButton.styleFrom(
                backgroundColor: AppColors.lightNavy,
                fixedSize: Size(322, 37),
                side: BorderSide(color: AppColors.blue, width: 0.66),
                shape: RoundedRectangleBorder(
                  borderRadius: BorderRadius.circular(0),
                ),
              ),
              onPressed: () {
                Navigator.pushNamed(context, ScreenConst.signInSignUpScreen);
              },
              child: Text(
                'SIGN UP',
                style: TextStyle(
                  color: AppColors.blue,
                  fontSize: 15,
                  fontFamily: 'Montserrat',
                ),
                textAlign: TextAlign.center,
              ),
            ),
          ],
        ),
      ),
    );
  }
}
