// import 'package:flutter/material.dart';
// import 'package:flutter_svg/flutter_svg.dart';
// import 'package:grad_project/core/const.dart';

// class SignInSignUpScreen extends StatelessWidget {
//   const SignInSignUpScreen({super.key});

//   @override
//   Widget build(BuildContext context) {
//     double width = AppSizes.screenWidth(context);
//     double height = AppSizes.screenHeight(context);
//     return Scaffold(
//       body: Center(
//         child: Column(
//           mainAxisAlignment: MainAxisAlignment.center,
//           children: [
//             Text(
//               'LINK',
//               style: TextStyle(
//                 color: AppColors.blue,
//                 fontSize: width * 0.06,
//                 fontFamily: 'Montserrat',
//               ),
//             ),
//             SizedBox(height: height * 0.06),
//             Container(
//               height: height * 0.055,
//               width: width * 0.89,
//               decoration: BoxDecoration(
//                 color: AppColors.lightNavy,
//                 border: Border.all(color: AppColors.blue, width: width * 0.003),
//                 borderRadius: BorderRadius.circular(width * 0.055),
//               ),
//               child: Center(
//                 child: Text(
//                   'Phone Number',
//                   style: TextStyle(
//                     color: AppColors.blue,
//                     fontSize: width * 0.04,
//                     fontFamily: 'Montserrat',
//                   ),
//                   textAlign: TextAlign.center,
//                 ),
//               ),
//             ),
//             SizedBox(height: height * 0.020),
//             Container(
//               height: height * 0.055,
//               width: width * 0.89,
//               decoration: BoxDecoration(
//                 color: AppColors.lightNavy,
//                 border: Border.all(color: AppColors.blue, width: width * 0.003),
//                 borderRadius: BorderRadius.circular(width * 0.055),
//               ),
//               child: Center(
//                 child: Text(
//                   'Password',
//                   style: TextStyle(
//                     color: AppColors.blue,
//                     fontSize: width * 0.04,
//                     fontFamily: 'Montserrat',
//                   ),
//                   textAlign: TextAlign.center,
//                 ),
//               ),
//             ),
//             SizedBox(height: height * 0.06),
//             ElevatedButton(
//               style: ElevatedButton.styleFrom(
//                 backgroundColor: AppColors.blue,
//                 shape: RoundedRectangleBorder(
//                   borderRadius: BorderRadius.circular(width * 0.055),
//                 ),
//                 minimumSize: Size(width * 0.89, height * 0.055),
//               ),
//               onPressed: () {
//                 Navigator.pushNamed(context, ScreenConst.mainScreen);
//               },
//               child: Text(
//                 'Submit',
//                 style: TextStyle(
//                   color: AppColors.white,
//                   fontSize: width * 0.04,
//                   fontFamily: 'Montserrat',
//                 ),
//                 textAlign: TextAlign.center,
//               ),
//             ),
//           ],
//         ),
//       ),
//     );
//   }
// }

import 'package:flutter/material.dart';
import 'package:flutter_bloc/flutter_bloc.dart';
import 'package:flutter_svg/svg.dart';
import 'package:grad_project/core/const.dart';
import 'package:grad_project/features/authentication/presentation/cubit/cubit/auth_cubit.dart';
import 'package:grad_project/features/authentication/presentation/cubit/cubit/auth_state.dart';

class SignInSignUpScreen extends StatefulWidget {
  const SignInSignUpScreen({super.key});

  @override
  State<SignInSignUpScreen> createState() => _SignInSignUpScreenState();
}

class _SignInSignUpScreenState extends State<SignInSignUpScreen> {
  final TextEditingController _phoneController = TextEditingController();
  final TextEditingController _passwordController = TextEditingController();

  bool _isLogin = true; // true = login, false = signUp

  @override
  Widget build(BuildContext context) {
    double width = AppSizes.screenWidth(context);
    double height = AppSizes.screenHeight(context);

    return Scaffold(
      body: Center(
        child: SingleChildScrollView(
          child: BlocConsumer<AuthCubit, AuthState>(
            listener: (context, state) {
              if (state is AuthSuccess) {
                // بعد تسجيل الدخول أو التسجيل نروح للصفحة الرئيسية
                Navigator.pushReplacementNamed(context, ScreenConst.mainScreen);
              } else if (state is AuthError) {
                ScaffoldMessenger.of(
                  context,
                ).showSnackBar(SnackBar(content: Text(state.message)));
              }
            },
            builder: (context, state) {
              bool isLoading = state is AuthLoading;

              return Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  SvgPicture.asset(
                    'assets/images/link_logo/link_logo.svg',
                    width: 97,
                    height: 120,
                  ),
                  SizedBox(height: height * 0.06),

                  // Phone TextField
                  Container(
                    height: height * 0.055,
                    width: width * 0.89,
                    decoration: BoxDecoration(
                      color: AppColors.lightNavy,
                      border: Border.all(
                        color: AppColors.blue,
                        width: width * 0.003,
                      ),
                      borderRadius: BorderRadius.circular(width * 0.055),
                    ),
                    child: TextField(
                      //textAlignVertical: TextAlignVertical.center,
                      controller: _phoneController,
                      keyboardType: TextInputType.phone,
                      decoration: InputDecoration(
                        border: InputBorder.none,
                        contentPadding: EdgeInsets.symmetric(
                          horizontal: 16,
                          vertical: 12,
                        ),
                        hintText: 'Phone Number',
                        hintStyle: TextStyle(
                          color: AppColors.blue,
                          fontSize: width * 0.04,
                        ),
                      ),
                      style: TextStyle(
                        color: AppColors.blue,
                        fontSize: width * 0.04,
                      ),
                    ),
                  ),
                  SizedBox(height: height * 0.020),

                  // Password TextField
                  Container(
                    height: height * 0.055,
                    width: width * 0.89,
                    decoration: BoxDecoration(
                      color: AppColors.lightNavy,
                      border: Border.all(
                        color: AppColors.blue,
                        width: width * 0.003,
                      ),
                      borderRadius: BorderRadius.circular(width * 0.055),
                    ),
                    child: TextField(
                      controller: _passwordController,
                      obscureText: true,
                      decoration: InputDecoration(
                        border: InputBorder.none,
                        contentPadding: EdgeInsets.symmetric(
                          horizontal: 16,
                          vertical: 12,
                        ),
                        hintText: 'Password',
                        hintStyle: TextStyle(
                          color: AppColors.blue,
                          fontSize: width * 0.04,
                        ),
                      ),
                      style: TextStyle(
                        color: AppColors.blue,
                        fontSize: width * 0.04,
                      ),
                    ),
                  ),
                  SizedBox(height: height * 0.06),

                  // Submit Button
                  ElevatedButton(
                    style: ElevatedButton.styleFrom(
                      backgroundColor: AppColors.blue,
                      shape: RoundedRectangleBorder(
                        borderRadius: BorderRadius.circular(width * 0.055),
                      ),
                      minimumSize: Size(width * 0.89, height * 0.055),
                    ),
                    onPressed: () {
                      Navigator.pushNamed(context, ScreenConst.mainScreen);
                    },
                    //  isLoading
                    //     ? null
                    //     : () {
                    //         final phone = _phoneController.text.trim();
                    //         final password = _passwordController.text.trim();

                    //         if (_isLogin) {
                    //           context.read<AuthCubit>().signin(
                    //                 phone: phone,
                    //                 password: password,
                    //               );
                    //         } else {
                    //           context.read<AuthCubit>().signUp(
                    //                 phone: phone,
                    //                 password: password,
                    //               );
                    //         }
                    //       },
                    child:
                    // isLoading
                    //     ? CircularProgressIndicator(color: AppColors.white)
                    //     :
                    Text(
                      _isLogin ? 'Login' : 'Sign Up',
                      style: TextStyle(
                        color: AppColors.white,
                        fontSize: width * 0.04,
                        fontFamily: 'Montserrat',
                      ),
                    ),
                  ),

                  SizedBox(height: 20),

                  // Switch between Login / SignUp
                  TextButton(
                    onPressed: () {
                      setState(() {
                        _isLogin = !_isLogin;
                      });
                    },
                    child: Text(
                      _isLogin
                          ? 'Don\'t have an account? Sign Up'
                          : 'Already have an account? Login',
                      style: TextStyle(
                        color: AppColors.blue,
                        fontSize: width * 0.035,
                      ),
                    ),
                  ),
                ],
              );
            },
          ),
        ),
      ),
    );
  }
}
