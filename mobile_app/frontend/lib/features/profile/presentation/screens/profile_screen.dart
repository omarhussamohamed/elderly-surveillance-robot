import 'package:flutter/material.dart';
import 'package:grad_project/core/const.dart';

class ProfileScreen extends StatelessWidget {
  const ProfileScreen({super.key});

  @override
  Widget build(BuildContext context) {
    double width = AppSizes.screenWidth(context);
    double height = AppSizes.screenHeight(context);
    return Scaffold(
      body: Center(
        child: Padding(
          padding: const EdgeInsets.symmetric(horizontal: 10),
          child: Column(
            children: [
              SizedBox(height: height * 0.09),

              Padding(
                padding: const EdgeInsets.symmetric(horizontal: 27),
                child: Container(
                  decoration: BoxDecoration(
                    borderRadius: BorderRadius.circular(width * 0.053),

                    border: Border.all(color: AppColors.grey),
                    color: AppColors.lightNavy,
                  ),
                  child: Column(
                    children: [
                      Padding(
                        padding: const EdgeInsets.symmetric(
                          vertical: 10,
                          horizontal: 10,
                        ),
                        child: Row(
                          children: [
                            Container(
                              width: width * 0.12,
                              height: width * 0.12,
                              decoration: BoxDecoration(
                                color: AppColors.paleBlue,
                                borderRadius: BorderRadius.circular(
                                  width * 0.08,
                                ),
                              ),
                              child: Icon(Icons.person, color: AppColors.blue),
                            ),
                            SizedBox(width: width * 0.056),
                            Expanded(
                              child: Column(
                                mainAxisAlignment: MainAxisAlignment.center,
                                crossAxisAlignment: CrossAxisAlignment.start,
                                children: [
                                  Text(
                                    'Elderly Name',
                                    style: TextStyle(
                                      color: AppColors.white,
                                      fontSize: width * 0.048,
                                      fontFamily: 'Montserrat',
                                    ),
                                  ),
                                  Text(
                                    '01012345678',
                                    style: TextStyle(
                                      color: AppColors.white,
                                      fontSize: width * 0.048,
                                      fontFamily: 'Montserrat',
                                    ),
                                    softWrap: true,
                                    maxLines: 2,
                                    overflow: TextOverflow.ellipsis,
                                  ),
                                ],
                              ),
                            ),

                            Icon(Icons.edit, color: AppColors.blue),
                          ],
                        ),
                      ),
                      Divider(color: AppColors.grey, thickness: 1),
                      Padding(
                        padding: const EdgeInsets.symmetric(horizontal: 12),
                        child: Column(
                          children: [
                            Align(
                              alignment: Alignment.topLeft,
                              child: Text(
                                'HOME ADDRESS',
                                style: TextStyle(
                                  color: AppColors.grey,
                                  fontSize: width * 0.032,
                                  fontFamily: 'Montserrat',
                                ),
                              ),
                            ),
                            Align(
                              alignment: Alignment.topLeft,
                              child: Text(
                                '23 Dr Mohammed Nagy, 6th of October',
                                style: TextStyle(
                                  color: AppColors.white,
                                  fontSize: width * 0.032,
                                  fontFamily: 'Montserrat',
                                ),
                              ),
                            ),
                            Align(
                              alignment: Alignment.topLeft,
                              child: Text(
                                'BIRTHDATE',
                                style: TextStyle(
                                  color: AppColors.grey,
                                  fontSize: width * 0.032,
                                  fontFamily: 'Montserrat',
                                ),
                              ),
                            ),
                            Align(
                              alignment: Alignment.topLeft,
                              child: Text(
                                '22 July 2003',
                                style: TextStyle(
                                  color: AppColors.white,
                                  fontSize: width * 0.032,
                                  fontFamily: 'Montserrat',
                                ),
                              ),
                            ),
                            Divider(color: AppColors.grey, thickness: 1),
                          ],
                        ),
                      ),
                      Column(
                        children: [
                          Align(
                            alignment: Alignment.topLeft,
                            child: Text(
                              'HOME ADDRESS',
                              style: TextStyle(
                                color: AppColors.grey,
                                fontSize: width * 0.032,
                                fontFamily: 'Montserrat',
                              ),
                            ),
                          ),
                          Align(
                            alignment: Alignment.topLeft,
                            child: Text(
                              '23 Dr Mohammed Nagy, 6th of October',
                              style: TextStyle(
                                color: AppColors.white,
                                fontSize: width * 0.032,
                                fontFamily: 'Montserrat',
                              ),
                            ),
                          ),
                          Align(
                            alignment: Alignment.topLeft,
                            child: Text(
                              'BIRTHDATE',
                              style: TextStyle(
                                color: AppColors.grey,
                                fontSize: width * 0.032,
                                fontFamily: 'Montserrat',
                              ),
                            ),
                          ),
                          Align(
                            alignment: Alignment.topLeft,
                            child: Text(
                              '22 July 2003',
                              style: TextStyle(
                                color: AppColors.white,
                                fontSize: width * 0.032,
                                fontFamily: 'Montserrat',
                              ),
                            ),
                          ),
                          Divider(color: AppColors.grey, thickness: 1),
                        ],
                      ),
                    ],
                  ),
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}




// Center(
//         child: Padding(
//           padding: const EdgeInsets.symmetric(horizontal: 48),
//           child: Column(
//             children: [
//               SizedBox(height: 50),
//               Text(
//                 'Profile',
//                 style: TextStyle(
//                   color: AppColors.white,
//                   fontSize: 21,
//                   fontFamily: 'Montserrat',
//                 ),
//               ),
//               SizedBox(height: 51),
//               Padding(
//                 padding: const EdgeInsets.symmetric(horizontal: 48),
//                 child: Row(
//                   children: [
//                     Container(
//                       width: 64,
//                       height: 64,
//                       decoration: BoxDecoration(
//                         shape: BoxShape.circle,
//                         color: AppColors.paleBlue,
//                       ),
//                     ),
//                     SizedBox(width: 21),
//                     Column(
//                       crossAxisAlignment: CrossAxisAlignment.start,
//                       children: [
//                         Text(
//                           'Suma Abbas',
//                           style: TextStyle(
//                             color: AppColors.white,
//                             fontSize: 16,
//                             fontFamily: 'Montserrat',
//                           ),
//                         ),
//                         Text(
//                           '01225589100',
//                           style: TextStyle(
//                             color: AppColors.white,
//                             fontSize: 16,
//                             fontFamily: 'Montserrat',
//                           ),
//                         ),
//                       ],
//                     ),
//                   ],
//                 ),
//               ),
//               Divider(color: AppColors.grey, thickness: 1),
//               SizedBox(height: 12),
//               Padding(
//                 padding: const EdgeInsets.only(right: 48),
//                 child: Text(
//                   'HOME ADDRESS',
//                   style: TextStyle(
//                     color: AppColors.grey,
//                     fontSize: 11,
//                     fontFamily: 'Montserrat',
//                   ),
//                 ),
//               ),
//               Padding(
//                 padding: const EdgeInsets.only(right: 48),
//                 child: Text(
//                   '23 Dr Mohammed Nagy, 6th of October City',
//                   style: TextStyle(
//                     color: AppColors.white,
//                     fontSize: 13,
//                     fontFamily: 'Montserrat',
//                   ),
//                 ),
//               ),
//               Padding(
//                 padding: const EdgeInsets.only(right: 48),
//                 child: Text(
//                   'BIRTHDATE',
//                   style: TextStyle(
//                     color: AppColors.grey,
//                     fontSize: 11,
//                     fontFamily: 'Montserrat',
//                   ),
//                 ),
//               ),
//               Padding(
//                 padding: const EdgeInsets.only(right: 48),
//                 child: Text(
//                   '22 July 2003',
//                   style: TextStyle(
//                     color: AppColors.white,
//                     fontSize: 13,
//                     fontFamily: 'Montserrat',
//                   ),
//                 ),
//               ),
//               Divider(color: AppColors.grey, thickness: 1),
//               SizedBox(height: 12),

//               Text(
//                 'MEDICAL CONDITIONS',
//                 style: TextStyle(
//                   color: AppColors.grey,
//                   fontSize: 11,
//                   fontFamily: 'Montserrat',
//                 ),
//               ),

//               Padding(
//                 padding: const EdgeInsets.only(right: 48),
//                 child: Text(
//                   'type 2 diabetes, hypertension',
//                   style: TextStyle(
//                     color: AppColors.white,
//                     fontSize: 13,
//                     fontFamily: 'Montserrat',
//                   ),
//                 ),
//               ),
//               Padding(
//                 padding: const EdgeInsets.only(right: 48),
//                 child: Text(
//                   'BLOOD TYPE',
//                   style: TextStyle(
//                     color: AppColors.grey,
//                     fontSize: 11,
//                     fontFamily: 'Montserrat',
//                   ),
//                 ),
//               ),
//               Padding(
//                 padding: const EdgeInsets.only(right: 48),
//                 child: Text(
//                   'O Positive O+',
//                   style: TextStyle(
//                     color: AppColors.white,
//                     fontSize: 13,
//                     fontFamily: 'Montserrat',
//                   ),
//                 ),
//               ),
//               Padding(
//                 padding: const EdgeInsets.only(right: 48),
//                 child: Text(
//                   'ALLERGIES',
//                   style: TextStyle(
//                     color: AppColors.grey,
//                     fontSize: 11,
//                     fontFamily: 'Montserrat',
//                   ),
//                 ),
//               ),
//               Text(
//                 'Penicillin, peanuts, Latex',
//                 style: TextStyle(
//                   color: AppColors.white,
//                   fontSize: 13,
//                   fontFamily: 'Montserrat',
//                 ),
//               ),
//               SizedBox(height: 12),
//               Padding(
//                 padding: const EdgeInsets.only(right: 48),
//                 child: Text(
//                   'MEDICATIONS',
//                   style: TextStyle(
//                     color: AppColors.grey,
//                     fontSize: 11,
//                     fontFamily: 'Montserrat',
//                   ),
//                 ),
//               ),
//               Padding(
//                 padding: const EdgeInsets.only(right: 48),
//                 child: Text(
//                   'Metformin (500mg), Lisinopril (10mg)',
//                   style: TextStyle(
//                     color: AppColors.white,
//                     fontSize: 13,
//                     fontFamily: 'Montserrat',
//                   ),
//                 ),
//               ),
//               Divider(color: AppColors.grey, thickness: 1),
//               SizedBox(height: 12),
//               Padding(
//                 padding: const EdgeInsets.only(right: 48),
//                 child: Text(
//                   'CARE INSTRUCTIONS',
//                   style: TextStyle(
//                     color: AppColors.grey,
//                     fontSize: 11,
//                     fontFamily: 'Montserrat',
//                   ),
//                 ),
//               ),
//               Padding(
//                 padding: const EdgeInsets.only(right: 48),
//                 child: Text(
//                   'In case of a fall, do not lift manually. Use the lift-assist chair in the hallway ',
//                   style: TextStyle(
//                     color: AppColors.white,
//                     fontSize: 13,
//                     fontFamily: 'Montserrat',
//                   ),
//                 ),
//               ),
//               SizedBox(height: 12),

//               Padding(
//                 padding: const EdgeInsets.only(right: 48),
//                 child: Text(
//                   'High fall risk. Check the "Safety  logs" robot alerts to inactivity',
//                   style: TextStyle(
//                     color: AppColors.white,
//                     fontSize: 13,
//                     fontFamily: 'Montserrat',
//                   ),
//                 ),
//               ),
//               SizedBox(height: 12),

//               Padding(
//                 padding: const EdgeInsets.only(right: 48),
//                 child: Text(
//                   'Requires medication reminders at 8:00 AM and 8:00 PM daily',
//                   style: TextStyle(
//                     color: AppColors.white,
//                     fontSize: 13,
//                     fontFamily: 'Montserrat',
//                   ),
//                 ),
//               ),
//             ],
//           ),
//         ),
//       ),