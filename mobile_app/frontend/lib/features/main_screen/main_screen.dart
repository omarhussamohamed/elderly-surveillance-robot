import 'package:flutter/material.dart';
import 'package:grad_project/core/const.dart';
import 'package:grad_project/features/home/presentation/screens/home_screen.dart';
import 'package:grad_project/features/home/presentation/screens/menu_screen.dart';
import 'package:grad_project/features/home/presentation/screens/remote_screen.dart';

class MainScreen extends StatefulWidget {
  const MainScreen({super.key});

  @override
  State<MainScreen> createState() => _MainScreenState();
}

class _MainScreenState extends State<MainScreen> {
  int _currentIndex = 0;

  final List<Widget> _pages = [HomeScreen(), RemoteScreen(), MenuScreen()];

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      // ------------------------------------------------------------------------------------------------------------  1  2   3
      body: _pages[_currentIndex],

      // ------------------------------------------------------------------------------------------------------------
      bottomNavigationBar: Container(
        decoration: const BoxDecoration(
          border: Border(top: BorderSide(color: AppColors.grey, width: 1)),
        ),
        child: BottomNavigationBar(
          currentIndex: _currentIndex,

          onTap: (index) {
            setState(() {
              _currentIndex = index;
            });
          },
          items: [
            BottomNavigationBarItem(
              backgroundColor: AppColors.lightNavy,
              icon: Icon(Icons.home),
              label: 'HOME',
              // icon: Image.asset(
              //   'assets/images/home_icon/home.png',
              //   width: 24,
              //   height: 24,
              // ),
            ),
            BottomNavigationBarItem(
              icon: Icon(Icons.wifi_tethering),
              label: 'REMOTE',

              // Image.asset(
              //   'assets/images/home_icon/home.png',

              //   width: 24,
              //   height: 24,
              // ),
              // label: 'Profile',
            ),
            BottomNavigationBarItem(
              icon: Icon(Icons.menu),
              label: 'MENU',

              // icon: Image.asset(
              //   'assets/images/home_icon/home.png',

              //   width: 24,
              //   height: 24,
              // ),
              // label: 'Settings',
            ),
          ],
        ),
      ),
    );
  }
}
