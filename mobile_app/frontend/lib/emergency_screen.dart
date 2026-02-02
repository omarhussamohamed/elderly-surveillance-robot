import 'package:flutter/material.dart';
import 'package:grad_project/core/const.dart';
//import 'package:gradapp/core/const.dart';

class EmergencyCall extends StatelessWidget {
  const EmergencyCall({super.key});

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
                  'Emergency Call',
                  style: TextStyle(
                    color: AppColors.white,
                    fontSize: width * 0.056,
                    fontFamily: 'Montserrat',
                  ),
                ),
                SizedBox(height: height * 0.0345),
                Container(
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
                          child: Icon(Icons.person, color: AppColors.blue),
                        ),
                        SizedBox(width: width * 0.056),
                        Expanded(
                          child: Text(
                            'Suma',
                            style: TextStyle(
                              color: AppColors.white,
                              fontSize: width * 0.048,
                              fontFamily: 'Montserrat',
                            ),
                          ),
                        ),

                        Icon(Icons.phone, color: AppColors.blue),
                      ],
                    ),
                  ),
                ),

                //--------------------------------------------------------------------------------------------------------
                SizedBox(height: width * 0.056),
                Container(
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
                            Icons.medical_services,
                            color: AppColors.blue,
                          ),
                        ),
                        SizedBox(width: width * 0.056),

                        Expanded(
                          child: Text(
                            'Medical Services',
                            style: TextStyle(
                              color: AppColors.white,
                              fontSize: width * 0.048,
                              fontFamily: 'Montserrat',
                            ),
                          ),
                        ),
                        Icon(Icons.phone, color: AppColors.blue),
                      ],
                    ),
                  ),
                ),
                //--------------------------------------------------------------------------------------------------------
                SizedBox(height: width * 0.056),
                Container(
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
                            Icons.fire_extinguisher_outlined,
                            color: AppColors.blue,
                          ),
                        ),
                        SizedBox(width: width * 0.056),
                        Expanded(
                          child: Text(
                            'Fire Department',
                            style: TextStyle(
                              color: AppColors.white,
                              fontSize: width * 0.048,
                              fontFamily: 'Montserrat',
                            ),
                          ),
                        ),

                        Icon(Icons.phone, color: AppColors.blue),
                      ],
                    ),
                  ),
                ),
                //--------------------------------------------------------------------------------------------------------
                SizedBox(height: width * 0.056),
                Container(
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
                          child: Icon(Icons.person, color: AppColors.blue),
                        ),
                        SizedBox(width: width * 0.056),
                        Expanded(
                          child: Text(
                            'Police',
                            style: TextStyle(
                              color: AppColors.white,
                              fontSize: width * 0.048,
                              fontFamily: 'Montserrat',
                            ),
                          ),
                        ),

                        Icon(Icons.phone, color: AppColors.blue),
                      ],
                    ),
                  ),
                ),

                Spacer(),
                ElevatedButton(
                  style: ElevatedButton.styleFrom(
                    backgroundColor: AppColors.lightNavy,
                    side: BorderSide(
                      color: AppColors.grey,
                      width: width * 0.0013,
                    ),
                    shape: RoundedRectangleBorder(
                      borderRadius: BorderRadius.circular(width * 0.08),
                    ),
                    minimumSize: Size(width * 0.307, height * 0.057),
                  ),
                  onPressed: () {
                    Navigator.pop(context);
                  },
                  child: Text(
                    'Back',
                    style: TextStyle(
                      color: AppColors.blue,
                      fontSize: width * 0.048,
                      fontFamily: 'Montserrat',
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
