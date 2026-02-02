import 'package:flutter/material.dart';
import 'package:flutter_svg/flutter_svg.dart';
import 'package:grad_project/core/const.dart';
//import 'package:gradapp/core/const.dart';

class HomeScreen extends StatefulWidget {
  const HomeScreen({super.key});

  @override
  State<HomeScreen> createState() => _HomeScreenState();
}

class _HomeScreenState extends State<HomeScreen> {
  @override
  Widget build(BuildContext context) {
    double width = AppSizes.screenWidth(context);
    double height = AppSizes.screenHeight(context);
    return Scaffold(
      backgroundColor: AppColors.darkNavy,
      appBar: AppBar(
        automaticallyImplyLeading: false,
        toolbarHeight: height * 0.125,
        backgroundColor: AppColors.lightNavy,
        shape: Border(
          bottom: BorderSide(color: AppColors.grey, width: width * 0.0027),
        ),
        title: Padding(
          padding: EdgeInsets.only(left: width * 0.013),
          child: Text(
            'Welcome Back :)',
            style: TextStyle(
              color: AppColors.white,
              fontFamily: 'Montserrat',
              fontSize: width * 0.053,
            ),
          ),
        ),
        actions: [
          Padding(
            padding: EdgeInsets.only(right: width * 0.101),
            child: IconButton(
              onPressed: () {
                Navigator.pushNamed(context, ScreenConst.notificationScreen);
              },
              icon: Icon(Icons.notifications, color: AppColors.blue),
            ),
            // SvgPicture.asset(
            //   'assets/images/notification_icon/notification.svg',
            //   width: width * 0.048,
            //   height: width * 0.048,
            //   key: const ValueKey('home_icon'),
            // ),
          ),
        ],
      ),
      body: SingleChildScrollView(
        child: Padding(
          padding: EdgeInsets.symmetric(
            horizontal: width * 0.05,
            vertical: height * 0.025,
          ),
          child: Column(
            children: [
              Row(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                children: [
                  Text(
                    'System Health',
                    style: TextStyle(
                      color: AppColors.white,
                      fontFamily: 'Montserrat',
                      fontSize: width * (20 / 375),
                    ),
                  ),
                  Text(
                    'CONNECTED',
                    style: TextStyle(
                      color: AppColors.green,
                      fontFamily: 'Montserrat',
                      fontSize: width * (11 / 375),
                    ),
                  ),
                ],
              ),
              SizedBox(height: height * (13 / 812)),
              Row(
                children: [
                  Container(
                    height: 41 * (height / 680),
                    width: 150 * (width / 375),
                    decoration: BoxDecoration(
                      color: AppColors.lightNavy,
                      borderRadius: BorderRadius.circular(width * 0.053),
                      border: Border.all(
                        color: AppColors.grey,
                        width: width * 0.0013,
                      ),
                    ),
                    child: Padding(
                      padding: EdgeInsets.symmetric(
                        horizontal: width * 0.045,
                        //vertical: height * 0.012,
                      ),
                      child: Row(
                        mainAxisAlignment: MainAxisAlignment.center,
                        children: [
                          Text(
                            'POWER',
                            style: TextStyle(
                              fontFamily: 'Montserrat',
                              fontSize: width * 0.04,
                              color: AppColors.blue,
                            ),
                          ),
                          SizedBox(width: width * 0.05),
                          Text(
                            '85%',
                            style: TextStyle(
                              fontSize: width * 0.037,
                              color: AppColors.white,
                              fontFamily: 'Montserrat',
                            ),
                          ),
                        ],
                      ),
                    ),
                  ),
                  SizedBox(width: 20),
                  Container(
                    height: 41 * (height / 680),
                    width: 150 * (width / 375),
                    decoration: BoxDecoration(
                      color: AppColors.lightNavy,
                      borderRadius: BorderRadius.circular(20),
                      border: Border.all(
                        color: AppColors.grey,
                        width: width * 0.0013,
                      ),
                    ),
                    child: Padding(
                      padding: EdgeInsets.symmetric(
                        horizontal: width * 0.045,
                        // vertical: 10,
                      ),
                      child: Row(
                        mainAxisAlignment: MainAxisAlignment.center,
                        children: [
                          Text(
                            'TEMP',
                            style: TextStyle(
                              fontFamily: 'Montserrat',
                              fontSize: width * 0.04,

                              color: AppColors.blue,
                            ),
                          ),
                          SizedBox(width: width * 0.05),

                          Text(
                            '24°C',
                            style: TextStyle(
                              fontSize: width * 0.037,
                              fontFamily: 'Montserrat',
                              color: AppColors.white,
                            ),
                          ),
                        ],
                      ),
                    ),
                  ),
                ],
              ),
              SizedBox(height: height * 0.028),
              Row(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                children: [
                  Text(
                    'Safety Monitor',
                    style: TextStyle(
                      color: AppColors.white,
                      fontFamily: 'Montserrat',
                      fontSize: width * 0.053,
                    ),
                  ),
                  Text(
                    'SAFE',
                    style: TextStyle(
                      color: AppColors.green,
                      fontFamily: 'Montserrat',
                      fontSize: width * 0.029,
                    ),
                  ),
                ],
              ),
              SizedBox(height: height * 0.02),
              Row(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  Container(
                    height: height * 0.230,
                    width: width * 0.41,
                    decoration: BoxDecoration(
                      color: AppColors.lightNavy,
                      borderRadius: BorderRadius.circular(width * 0.053),
                      border: Border.all(
                        color: AppColors.green,
                        width: width * 0.0013,
                      ),
                    ),
                    child: Column(
                      mainAxisAlignment: MainAxisAlignment.center,
                      children: [
                        Container(
                          width: width * 0.12,
                          height: width * 0.12,
                          decoration: BoxDecoration(
                            borderRadius: BorderRadius.circular(width * 0.08),
                            color: AppColors.paleGreen,
                          ),
                          //child: Icon(Icons.cloud, color: AppColors.green),
                          child: Image.asset(
                            'assets/gass.png',
                            width: 20,
                            height: 20,
                          ),
                          //child:
                          //  SvgPicture.asset(
                          //   'assets/images/gas_icon/gas.svg',
                          //   width: 20,
                          //   height: 20,
                          // // ),
                          // Image.asset(
                          //   'assets/images/notification_icon/gass.jpg',
                          //   width: 20,
                          //   height: 20,
                          // ),
                        ),
                        Padding(
                          padding: EdgeInsets.only(top: height * 0.01),
                          child: Text(
                            'GAS',
                            style: TextStyle(
                              fontFamily: 'Montserrat',
                              fontSize: width * 0.029,
                              color: AppColors.grey,
                            ),
                          ),
                        ),
                        Text(
                          'Normal',
                          style: TextStyle(
                            fontFamily: 'Montserrat',
                            fontSize: width * 0.053,
                            color: AppColors.white,
                          ),
                        ),
                      ],
                    ),
                  ),
                  SizedBox(width: width * 0.053),
                  Container(
                    height: height * 0.230,
                    width: width * 0.41,
                    decoration: BoxDecoration(
                      color: AppColors.lightNavy,
                      borderRadius: BorderRadius.circular(width * 0.053),
                      border: Border.all(
                        color: AppColors.green,
                        width: width * 0.0013,
                      ),
                    ),
                    child: Column(
                      mainAxisAlignment: MainAxisAlignment.center,
                      children: [
                        Container(
                          width: width * 0.12,
                          height: width * 0.12,
                          decoration: BoxDecoration(
                            borderRadius: BorderRadius.circular(width * 0.08),
                            color: AppColors.paleGreen,
                          ),
                          // child: Icon(
                          //   Icons.local_fire_department,
                          //   color: AppColors.green,
                          // ),
                          child: Image.asset(
                            'assets/fire.png',
                            width: 20,
                            height: 20,
                          ),
                          // child: SvgPicture.asset(
                          //   'assets/fire.svg',
                          //   width: 10,
                          //   height: 10,
                          //   // fit: BoxFit.contain,
                          // ),
                          // Image.asset(
                          //   'assets/images/gas.png',
                          //   width: 20,
                          //   height: 20,
                          // ),
                        ),
                        Padding(
                          padding: EdgeInsets.only(top: height * 0.01),
                          child: Text(
                            'FIRE',
                            style: TextStyle(
                              fontFamily: 'Montserrat',
                              fontSize: width * 0.029,
                              color: AppColors.grey,
                            ),
                          ),
                        ),
                        Text(
                          'None',
                          style: TextStyle(
                            fontFamily: 'Montserrat',
                            fontSize: width * 0.053,
                            color: AppColors.white,
                          ),
                        ),
                      ],
                    ),
                  ),
                ],
              ),
              SizedBox(height: height * 0.025),
              Row(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  Container(
                    height: height * 0.230,
                    width: width * 0.41,
                    decoration: BoxDecoration(
                      color: AppColors.lightNavy,
                      borderRadius: BorderRadius.circular(width * 0.053),
                      border: Border.all(
                        color: AppColors.green,
                        width: width * 0.0013,
                      ),
                    ),
                    child: Column(
                      mainAxisAlignment: MainAxisAlignment.center,
                      children: [
                        Container(
                          width: width * 0.12,
                          height: width * 0.12,
                          decoration: BoxDecoration(
                            borderRadius: BorderRadius.circular(width * 0.08),
                            color: AppColors.paleGreen,
                          ),
                          // child: SvgPicture.asset(
                          //   'assets/fall.svg',
                          //   width: 10,
                          //   height: 10,
                          //   //  fit: BoxFit.contain,
                          // ),
                          // child: Icon(
                          //   Icons.warning_amber_rounded,
                          //   color: AppColors.green,
                          // ),
                          // child: SvgPicture.asset(
                          //   'assets/images/fall_icon/fall.svg',
                          //   width: 20,
                          //   height: 20,
                          // // ),
                          child: Image.asset(
                            'assets/fall.png',
                            width: 20,
                            height: 20,
                          ),
                        ),
                        Padding(
                          padding: EdgeInsets.only(top: height * 0.01),
                          child: Text(
                            'FALL',
                            style: TextStyle(
                              fontFamily: 'Montserrat',
                              fontSize: width * 0.029,
                              color: AppColors.grey,
                            ),
                          ),
                        ),
                        Text(
                          'None',
                          style: TextStyle(
                            fontFamily: 'Montserrat',
                            fontSize: width * 0.053,
                            color: AppColors.white,
                          ),
                        ),
                      ],
                    ),
                  ),
                  SizedBox(width: width * 0.053),
                  Container(
                    height: height * 0.230,
                    width: width * 0.41,
                    decoration: BoxDecoration(
                      color: AppColors.lightNavy,
                      borderRadius: BorderRadius.circular(width * 0.053),
                      border: Border.all(
                        color: AppColors.green,
                        width: width * 0.0013,
                      ),
                    ),
                    child: Column(
                      mainAxisAlignment: MainAxisAlignment.center,
                      children: [
                        Container(
                          width: width * 0.12,
                          height: width * 0.12,
                          decoration: BoxDecoration(
                            borderRadius: BorderRadius.circular(width * 0.08),
                            color: AppColors.paleGreen,
                          ),
                          // child: Icon(
                          //   Icons.person_2_outlined,
                          //   color: AppColors.green,
                          // ),
                          // child: SvgPicture.asset(
                          //   'assets/gas.svg',
                          //   width: 20,
                          //   height: 20,
                          // ),
                          child: Image.asset(
                            'assets/stranger.png',
                            width: 20,
                            height: 20,
                          ),
                        ),
                        Padding(
                          padding: EdgeInsets.only(top: height * 0.01),
                          child: Text(
                            'STRANGER',
                            style: TextStyle(
                              fontFamily: 'Montserrat',
                              fontSize: width * 0.029,
                              color: AppColors.grey,
                            ),
                          ),
                        ),
                        Text(
                          'None',
                          style: TextStyle(
                            fontFamily: 'Montserrat',
                            fontSize: width * 0.053,
                            color: AppColors.white,
                          ),
                        ),
                      ],
                    ),
                  ),
                ],
              ),
              SizedBox(height: height * 0.025),
              Container(
                height: height * 0.1,
                width: width * 0.875,
                decoration: BoxDecoration(
                  color: AppColors.lightNavy,
                  borderRadius: BorderRadius.circular(width * 0.053),
                  border: Border.all(
                    color: AppColors.grey,
                    width: width * 0.0013,
                  ),
                ),
                child: Padding(
                  padding: EdgeInsets.symmetric(
                    horizontal: width * 0.045,
                    vertical: height * 0.015,
                  ),
                  child: Column(
                    children: [
                      Align(
                        alignment: Alignment.topLeft,
                        child: Text(
                          'LAST EVENT',
                          style: TextStyle(
                            fontFamily: 'Montserrat',
                            fontSize: width * 0.029,
                            color: AppColors.grey,
                          ),
                        ),
                      ),
                      SizedBox(height: height * 0.006),
                      Row(
                        mainAxisAlignment: MainAxisAlignment.spaceBetween,
                        children: [
                          Text(
                            'Fall detected in Hallway',
                            style: TextStyle(
                              fontFamily: 'Montserrat',
                              fontSize: width * 0.032,
                              color: AppColors.white,
                            ),
                          ),
                          Text(
                            '2:45 AM',
                            style: TextStyle(
                              fontFamily: 'Montserrat',
                              fontSize: width * 0.029,
                              color: AppColors.grey,
                            ),
                          ),
                        ],
                      ),
                    ],
                  ),
                ),
              ),
              SizedBox(height: height * 0.047),
              ElevatedButton(
                style: ElevatedButton.styleFrom(
                  backgroundColor: AppColors.lightNavy,
                  // borderRadius: BorderRadius.circular(width * 0.053),
                  side: BorderSide(color: AppColors.red, width: width * 0.0013),
                  minimumSize: Size(width * 0.52, height * 0.057),
                ),
                onPressed: () {
                  Navigator.pushNamed(context, ScreenConst.emergencyCallScreen);
                },
                child: Text(
                  'Emergency Call',
                  style: TextStyle(
                    fontFamily: 'Montserrat',
                    fontSize: width * 0.04,
                    color: AppColors.red,
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
