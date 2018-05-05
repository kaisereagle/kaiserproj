# DO NOT EDIT
# This makefile makes sure all linkable targets are
# up-to-date with anything they link to
default:
	echo "Do not invoke directly"

# Rules to remove targets that are older than anything to which they
# link.  This forces Xcode to relink the targets from scratch.  It
# does not seem to check these dependencies itself.
PostBuild.BulletSoftBody.Debug:
/Users/kaiser/work/lib/proj/BulletSoftBody/Debug/libBulletSoftBody.a:
	/bin/rm -f /Users/kaiser/work/lib/proj/BulletSoftBody/Debug/libBulletSoftBody.a


PostBuild.BulletCollision.Debug:
/Users/kaiser/work/lib/proj/BulletCollision/Debug/libBulletCollision.a:
	/bin/rm -f /Users/kaiser/work/lib/proj/BulletCollision/Debug/libBulletCollision.a


PostBuild.BulletDynamics.Debug:
/Users/kaiser/work/lib/proj/BulletDynamics/Debug/libBulletDynamics.a:
	/bin/rm -f /Users/kaiser/work/lib/proj/BulletDynamics/Debug/libBulletDynamics.a


PostBuild.LinearMath.Debug:
/Users/kaiser/work/lib/proj/LinearMath/Debug/libLinearMath.a:
	/bin/rm -f /Users/kaiser/work/lib/proj/LinearMath/Debug/libLinearMath.a


PostBuild.BulletSoftBody.Release:
/Users/kaiser/work/lib/proj/BulletSoftBody/Release/libBulletSoftBody.a:
	/bin/rm -f /Users/kaiser/work/lib/proj/BulletSoftBody/Release/libBulletSoftBody.a


PostBuild.BulletCollision.Release:
/Users/kaiser/work/lib/proj/BulletCollision/Release/libBulletCollision.a:
	/bin/rm -f /Users/kaiser/work/lib/proj/BulletCollision/Release/libBulletCollision.a


PostBuild.BulletDynamics.Release:
/Users/kaiser/work/lib/proj/BulletDynamics/Release/libBulletDynamics.a:
	/bin/rm -f /Users/kaiser/work/lib/proj/BulletDynamics/Release/libBulletDynamics.a


PostBuild.LinearMath.Release:
/Users/kaiser/work/lib/proj/LinearMath/Release/libLinearMath.a:
	/bin/rm -f /Users/kaiser/work/lib/proj/LinearMath/Release/libLinearMath.a


PostBuild.BulletSoftBody.MinSizeRel:
/Users/kaiser/work/lib/proj/BulletSoftBody/MinSizeRel/libBulletSoftBody.a:
	/bin/rm -f /Users/kaiser/work/lib/proj/BulletSoftBody/MinSizeRel/libBulletSoftBody.a


PostBuild.BulletCollision.MinSizeRel:
/Users/kaiser/work/lib/proj/BulletCollision/MinSizeRel/libBulletCollision.a:
	/bin/rm -f /Users/kaiser/work/lib/proj/BulletCollision/MinSizeRel/libBulletCollision.a


PostBuild.BulletDynamics.MinSizeRel:
/Users/kaiser/work/lib/proj/BulletDynamics/MinSizeRel/libBulletDynamics.a:
	/bin/rm -f /Users/kaiser/work/lib/proj/BulletDynamics/MinSizeRel/libBulletDynamics.a


PostBuild.LinearMath.MinSizeRel:
/Users/kaiser/work/lib/proj/LinearMath/MinSizeRel/libLinearMath.a:
	/bin/rm -f /Users/kaiser/work/lib/proj/LinearMath/MinSizeRel/libLinearMath.a


PostBuild.BulletSoftBody.RelWithDebInfo:
/Users/kaiser/work/lib/proj/BulletSoftBody/RelWithDebInfo/libBulletSoftBody.a:
	/bin/rm -f /Users/kaiser/work/lib/proj/BulletSoftBody/RelWithDebInfo/libBulletSoftBody.a


PostBuild.BulletCollision.RelWithDebInfo:
/Users/kaiser/work/lib/proj/BulletCollision/RelWithDebInfo/libBulletCollision.a:
	/bin/rm -f /Users/kaiser/work/lib/proj/BulletCollision/RelWithDebInfo/libBulletCollision.a


PostBuild.BulletDynamics.RelWithDebInfo:
/Users/kaiser/work/lib/proj/BulletDynamics/RelWithDebInfo/libBulletDynamics.a:
	/bin/rm -f /Users/kaiser/work/lib/proj/BulletDynamics/RelWithDebInfo/libBulletDynamics.a


PostBuild.LinearMath.RelWithDebInfo:
/Users/kaiser/work/lib/proj/LinearMath/RelWithDebInfo/libLinearMath.a:
	/bin/rm -f /Users/kaiser/work/lib/proj/LinearMath/RelWithDebInfo/libLinearMath.a




# For each target create a dummy ruleso the target does not have to exist
