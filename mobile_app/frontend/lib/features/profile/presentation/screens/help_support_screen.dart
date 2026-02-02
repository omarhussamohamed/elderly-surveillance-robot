import 'package:flutter/material.dart';
import 'package:grad_project/core/const.dart';

class HelpSupportScreen extends StatelessWidget {
  const HelpSupportScreen({super.key});

  @override
  Widget build(BuildContext context) {
    double width = AppSizes.screenWidth(context);
    double height = AppSizes.screenHeight(context);
    return Scaffold(
      body: SingleChildScrollView(
        child: Center(
          child: Padding(
            padding: EdgeInsets.symmetric(horizontal: 20, vertical: 20),
            child: Column(
              children: [
                SizedBox(height: height * 0.09),
                Text(
                  'Help & Support',
                  style: TextStyle(
                    color: AppColors.white,
                    fontSize: width * 0.056,
                    fontFamily: 'Montserrat',
                  ),
                ),
                SizedBox(height: height * 0.035),
                Container(
                  decoration: BoxDecoration(
                    color: AppColors.darkNavy,
                    border: Border.all(
                      color: AppColors.grey,
                      width: width * 0.002,
                    ),
                    borderRadius: BorderRadius.circular(width * 0.053),
                  ),
                  child: Padding(
                    padding: const EdgeInsets.symmetric(vertical: 10),
                    child: Column(
                      children: [
                        Text(
                          'User Manual',
                          style: TextStyle(
                            fontFamily: 'Montserrat',
                            color: AppColors.white,
                            fontSize: width * 0.05,
                          ),
                        ),
                        Divider(
                          color: AppColors.grey,
                          thickness: width * 0.002,
                        ),
                        Padding(
                          padding: const EdgeInsets.symmetric(horizontal: 20),
                          child: Column(
                            children: [
                              Text(
                                'Getting Started: "How to connect your robot to Wi-Fi"',
                                style: TextStyle(
                                  fontFamily: 'Montserrat',
                                  color: AppColors.white,
                                  fontSize: width * 0.036,
                                ),
                              ),
                              SizedBox(height: height * 0.025),
                              Text(
                                'Voice Commands: "A list of phrases your robot understands."',
                                style: TextStyle(
                                  fontFamily: 'Montserrat',
                                  color: AppColors.white,
                                  fontSize: width * 0.036,
                                ),
                              ),
                              SizedBox(height: height * 0.025),
                              Text(
                                'Safety Monitoring: "Understanding gas, fire, and fall alerts."',
                                style: TextStyle(
                                  fontFamily: 'Montserrat',
                                  color: AppColors.white,
                                  fontSize: width * 0.036,
                                ),
                              ),
                              SizedBox(height: height * 0.025),
                              Text(
                                'Changing Guide: "How to ensure the robot stays powered."',
                                style: TextStyle(
                                  fontFamily: 'Montserrat',
                                  color: AppColors.white,
                                  fontSize: width * 0.036,
                                ),
                              ),
                            ],
                          ),
                        ),
                      ],
                    ),
                  ),
                ),
                SizedBox(height: height * 0.043),
                Container(
                  decoration: BoxDecoration(
                    color: AppColors.darkNavy,
                    border: Border.all(
                      color: AppColors.grey,
                      width: width * 0.002,
                    ),
                    borderRadius: BorderRadius.circular(width * 0.053),
                  ),
                  child: Padding(
                    padding: const EdgeInsets.symmetric(vertical: 10),
                    child: Column(
                      children: [
                        Text(
                          'Common Question & Steps',
                          style: TextStyle(
                            fontFamily: 'Montserrat',
                            color: AppColors.white,
                            fontSize: width * 0.05,
                          ),
                        ),
                        Divider(
                          color: AppColors.grey,
                          thickness: width * 0.002,
                        ),
                        Padding(
                          padding: const EdgeInsets.symmetric(horizontal: 20),
                          child: Column(
                            children: [
                              Text(
                                'What if the robot gets stuck? \n"Follow these 3 steps to reset its pathing."',
                                style: TextStyle(
                                  fontFamily: 'Montserrat',
                                  color: AppColors.white,
                                  fontSize: width * 0.036,
                                ),
                              ),
                              SizedBox(height: height * 0.025),
                              Text(
                                'How do i update a face? \n"Step-by-step guide to improving recognetion."',
                                style: TextStyle(
                                  fontFamily: 'Montserrat',
                                  color: AppColors.white,
                                  fontSize: width * 0.036,
                                ),
                              ),
                              SizedBox(height: height * 0.025),
                              Text(
                                'Emergency button not work? \n"Manual override and troubleshooting steps."',
                                style: TextStyle(
                                  fontFamily: 'Montserrat',
                                  color: AppColors.white,
                                  fontSize: width * 0.036,
                                ),
                              ),
                            ],
                          ),
                        ),
                      ],
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
