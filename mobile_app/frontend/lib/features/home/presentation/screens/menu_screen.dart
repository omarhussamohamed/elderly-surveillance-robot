import 'package:flutter/material.dart';
import 'package:grad_project/core/const.dart';
//import 'package:gradapp/core/const.dart';

class MenuScreen extends StatelessWidget {
  const MenuScreen({super.key});

  @override
  Widget build(BuildContext context) {
    double width = AppSizes.screenWidth(context);
    double height = AppSizes.screenHeight(context);
    return Scaffold(
      backgroundColor: AppColors.darkNavy,
      body: Center(
        child: Padding(
          padding: EdgeInsets.symmetric(vertical: height * 0.0123),
          child: Padding(
            padding: EdgeInsets.symmetric(vertical: height * 0.042),
            child: Column(
              children: [
                Text(
                  'Menu',
                  style: TextStyle(
                    color: AppColors.white,
                    fontSize: width * 0.056,
                    fontFamily: 'Montserrat',
                  ),
                ),
                SizedBox(height: height * 0.0345),
                InkWell(
                  onTap: () {
                    Navigator.pushNamed(context, ScreenConst.profileScreen);
                  },
                  child: Container(
                    height: height * 0.12,
                    width: width * 0.92,
                    decoration: BoxDecoration(
                      color: AppColors.lightNavy,
                      border: Border.all(
                        color: AppColors.grey,
                        width: width * 0.0018,
                      ),
                      borderRadius: BorderRadius.circular(width * 0.053),
                    ),

                    child: Padding(
                      padding: EdgeInsets.symmetric(horizontal: width * 0.053),
                      child: Row(
                        children: [
                          Container(
                            width: width * 0.12,
                            height: width * 0.12,
                            decoration: BoxDecoration(
                              color: AppColors.paleBlue,
                              borderRadius: BorderRadius.circular(width * 0.08),
                            ),
                            child: Icon(
                              Icons.person,
                              color: AppColors.blue,
                              size: 30,
                            ),
                          ),
                          SizedBox(width: width * 0.056),
                          Expanded(
                            child: Column(
                              mainAxisAlignment: MainAxisAlignment.center,
                              crossAxisAlignment: CrossAxisAlignment.start,
                              children: [
                                Text(
                                  'Profile',
                                  style: TextStyle(
                                    color: AppColors.white,
                                    fontSize: width * 0.048,
                                    fontFamily: 'Montserrat',
                                  ),
                                ),
                                Text(
                                  'View elder ID, home address, and emergency info',
                                  style: TextStyle(
                                    color: AppColors.grey,
                                    fontSize: width * 0.032,
                                  ),
                                  softWrap: true,
                                  maxLines: 2,
                                  overflow: TextOverflow.ellipsis,
                                ),
                              ],
                            ),
                          ),

                          Icon(Icons.arrow_forward_ios, color: AppColors.blue),
                        ],
                      ),
                    ),
                  ),
                ),

                //--------------------------------------------------------------------------------------------------------
                SizedBox(height: width * 0.056),

                InkWell(
                  onTap: () {
                    Navigator.pushNamed(
                      context,
                      ScreenConst.managePeopleScreen,
                    );
                  },
                  child: Container(
                    height: height * 0.12,
                    width: width * 0.92,
                    decoration: BoxDecoration(
                      color: AppColors.lightNavy,
                      border: Border.all(
                        color: AppColors.grey,
                        width: width * 0.0018,
                      ),
                      borderRadius: BorderRadius.circular(width * 0.053),
                    ),

                    child: Padding(
                      padding: EdgeInsets.symmetric(horizontal: width * 0.053),
                      child: Row(
                        children: [
                          Container(
                            width: width * 0.12,
                            height: width * 0.12,
                            decoration: BoxDecoration(
                              color: AppColors.paleBlue,
                              borderRadius: BorderRadius.circular(width * 0.08),
                            ),
                            child: Icon(
                              Icons.person,
                              color: AppColors.blue,
                              size: 30,
                            ),
                          ),
                          SizedBox(width: width * 0.056),

                          Expanded(
                            child: Column(
                              mainAxisAlignment: MainAxisAlignment.center,
                              crossAxisAlignment: CrossAxisAlignment.start,
                              children: [
                                Text(
                                  'Manage People',
                                  style: TextStyle(
                                    color: AppColors.white,
                                    fontSize: width * 0.048,
                                    fontFamily: 'Montserrat',
                                  ),
                                ),
                                Text(
                                  'Add or remove people for face recognition',
                                  style: TextStyle(
                                    color: AppColors.grey,
                                    fontSize: width * 0.032,
                                  ),
                                  softWrap: true,
                                  maxLines: 2,
                                  overflow: TextOverflow.ellipsis,
                                ),
                              ],
                            ),
                          ),
                          Icon(Icons.arrow_forward_ios, color: AppColors.blue),
                        ],
                      ),
                    ),
                  ),
                ),
                //--------------------------------------------------------------------------------------------------------
                SizedBox(height: width * 0.056),
                InkWell(
                  onTap: () {
                    Navigator.pushNamed(context, ScreenConst.HelpSupportScreen);
                  },
                  child: Container(
                    height: height * 0.12,
                    width: width * 0.92,
                    decoration: BoxDecoration(
                      color: AppColors.lightNavy,
                      border: Border.all(
                        color: AppColors.grey,
                        width: width * 0.0018,
                      ),
                      borderRadius: BorderRadius.circular(width * 0.053),
                    ),

                    child: Padding(
                      padding: EdgeInsets.symmetric(horizontal: width * 0.053),
                      child: Row(
                        children: [
                          Container(
                            width: width * 0.12,
                            height: width * 0.12,
                            decoration: BoxDecoration(
                              color: AppColors.paleBlue,
                              borderRadius: BorderRadius.circular(width * 0.08),
                            ),
                            child: Icon(
                              Icons.person,
                              color: AppColors.blue,
                              size: 30,
                            ),
                          ),
                          SizedBox(width: width * 0.056),
                          Expanded(
                            child: Column(
                              mainAxisAlignment: MainAxisAlignment.center,
                              crossAxisAlignment: CrossAxisAlignment.start,
                              children: [
                                Text(
                                  'Help & Support',
                                  style: TextStyle(
                                    color: AppColors.white,
                                    fontSize: width * 0.048,
                                    fontFamily: 'Montserrat',
                                  ),
                                ),
                                Text(
                                  'User guide and troubleshooting tips',
                                  style: TextStyle(
                                    color: AppColors.grey,
                                    fontSize: width * 0.032,
                                  ),
                                  softWrap: true,
                                  maxLines: 2,
                                  overflow: TextOverflow.ellipsis,
                                ),
                              ],
                            ),
                          ),

                          Icon(Icons.arrow_forward_ios, color: AppColors.blue),
                        ],
                      ),
                    ),
                  ),
                ),
                Spacer(),
                Container(
                  height: height * 0.057,
                  width: width * 0.307,
                  decoration: BoxDecoration(
                    color: AppColors.lightNavy,
                    border: Border.all(
                      color: AppColors.red,
                      width: width * 0.0018,
                    ),
                    borderRadius: BorderRadius.circular(width * 0.08),
                  ),
                  child: Center(
                    child: Text(
                      'Log Out',
                      style: TextStyle(
                        color: AppColors.red,
                        fontSize: width * 0.048,
                        fontFamily: 'Montserrat',
                      ),
                    ),
                  ),
                ),
              ],
            ),
          ),
        ),
      ),
    );
  }
}
