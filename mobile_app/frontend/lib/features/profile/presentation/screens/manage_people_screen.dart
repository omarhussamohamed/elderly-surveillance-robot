import 'package:flutter/material.dart';
import 'package:grad_project/core/const.dart';

class ManagePeopleScreen extends StatelessWidget {
  ManagePeopleScreen({super.key});

  final List<String> People = [
    'Person 1',
    'Person 2',
    'Person 3',
    'Person 4',
    'Person 5',
    'Person 6',
    'Person 7',
    'Person 8',
    'Person 9',
    'Person 10',
    'Person 11',
    'Person 12',
  ];

  @override
  Widget build(BuildContext context) {
    double width = AppSizes.screenWidth(context);
    double height = AppSizes.screenHeight(context);
    return Scaffold(
      body: Center(
        child: Padding(
          padding: const EdgeInsets.symmetric(horizontal: 30),
          child: Column(
            children: [
              SizedBox(height: height * 0.09),
              Text(
                'Manage People',
                style: TextStyle(
                  color: AppColors.white,
                  fontSize: width * 0.056,
                  fontFamily: 'Montserrat',
                ),
              ),
              SizedBox(height: height * 0.035),

              //------------------------------------------------------------------------------
              // Expanded(
              //   child: GridView.builder(
              //     shrinkWrap: true,
              //     physics: NeverScrollableScrollPhysics(),
              //     itemCount: People.length,
              //     gridDelegate: SliverGridDelegateWithFixedCrossAxisCount(
              //       crossAxisCount: 3, // 3 columns

              //       crossAxisSpacing: width * 0.05,
              //       mainAxisSpacing: height * 0.02,
              //       childAspectRatio: 0.9, // adjust to fit container + text
              //     ),
              //     itemBuilder: (ctx, index) {
              //       final person = People[index];
              //       return GestureDetector(
              //         onTap: () {
              //           print('$person clicked!');
              //         },
              //         child: Column(
              //           children: [
              //             Container(
              //               width: width * 0.2, // responsive size
              //               height: width * 0.2, // make it square
              //               decoration: BoxDecoration(
              //                 color: AppColors.lightNavy,
              //                 border: Border.all(color: AppColors.grey),
              //                 borderRadius: BorderRadius.circular(
              //                   width * 0.053,
              //                 ),
              //               ),
              //             ),
              //             SizedBox(height: height * 0.01),
              //             Text(
              //               person, // name under the container
              //               textAlign: TextAlign.center,
              //               style: TextStyle(
              //                 color: AppColors.white,
              //                 fontSize: width * 0.036,
              //                 fontFamily: 'Montserrat',
              //               ),
              //             ),
              //           ],
              //         ),
              //       );
              //     },
              //   ),
              // ),
              GridView.builder(
                shrinkWrap: true,
                physics: BouncingScrollPhysics(),
                itemCount: People.length,
                gridDelegate: SliverGridDelegateWithFixedCrossAxisCount(
                  crossAxisCount: 3,
                  crossAxisSpacing: 20,
                  mainAxisSpacing: 30,
                  childAspectRatio: 1.1,
                ),
                itemBuilder: (ctx, index) {
                  final singlePerson = People[index];
                  return Column(
                    children: [
                      Container(
                        width: 76,
                        height: 76,
                        decoration: BoxDecoration(
                          color: AppColors.lightNavy,
                          border: Border.all(color: AppColors.grey),
                          borderRadius: BorderRadius.circular(width * 0.053),
                        ),
                        child: Icon(
                          Icons.person,
                          color: AppColors.blue,
                          size: 40,
                        ),
                      ),

                      // SizedBox(height: 10),
                      // Text(
                      //   singlePerson,
                      //   textAlign: TextAlign.center,
                      //   style: TextStyle(
                      //     color: AppColors.white,
                      //     fontSize: width * 0.034,
                      //     fontFamily: 'Montserrat',
                      //   ),
                      //   maxLines: 2,
                      //   overflow: TextOverflow.ellipsis,
                      // ),
                    ],
                  );
                },
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
                  'Add Person',
                  style: TextStyle(
                    color: AppColors.blue,
                    fontSize: width * 0.048,
                    fontFamily: 'Montserrat',
                  ),
                ),
              ),
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
    );
  }
}
