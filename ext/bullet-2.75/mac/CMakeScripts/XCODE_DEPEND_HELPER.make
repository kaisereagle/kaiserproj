# DO NOT EDIT
# This makefile makes sure all linkable targets are
# up-to-date with anything they link to
default:
	echo "Do not invoke directly"

# Rules to remove targets that are older than anything to which they
# link.  This forces Xcode to relink the targets from scratch.  It
# does not seem to check these dependencies itself.
PostBuild.BulletSoftBody.Debug:
/Users/sakya-suzuki/work/kaiserproj/ext/bullet-2.75/mac/src/BulletSoftBody/Debug/libBulletSoftBody.a:
	/bin/rm -f /Users/sakya-suzuki/work/kaiserproj/ext/bullet-2.75/mac/src/BulletSoftBody/Debug/libBulletSoftBody.a


PostBuild.BulletCollision.Debug:
/Users/sakya-suzuki/work/kaiserproj/ext/bullet-2.75/mac/src/BulletCollision/Debug/libBulletCollision.a:
	/bin/rm -f /Users/sakya-suzuki/work/kaiserproj/ext/bullet-2.75/mac/src/BulletCollision/Debug/libBulletCollision.a


PostBuild.BulletDynamics.Debug:
/Users/sakya-suzuki/work/kaiserproj/ext/bullet-2.75/mac/src/BulletDynamics/Debug/libBulletDynamics.a:
	/bin/rm -f /Users/sakya-suzuki/work/kaiserproj/ext/bullet-2.75/mac/src/BulletDynamics/Debug/libBulletDynamics.a


PostBuild.LinearMath.Debug:
/Users/sakya-suzuki/work/kaiserproj/ext/bullet-2.75/mac/src/LinearMath/Debug/libLinearMath.a:
	/bin/rm -f /Users/sakya-suzuki/work/kaiserproj/ext/bullet-2.75/mac/src/LinearMath/Debug/libLinearMath.a


PostBuild.BulletSoftBody.Release:
/Users/sakya-suzuki/work/kaiserproj/ext/bullet-2.75/mac/src/BulletSoftBody/Release/libBulletSoftBody.a:
	/bin/rm -f /Users/sakya-suzuki/work/kaiserproj/ext/bullet-2.75/mac/src/BulletSoftBody/Release/libBulletSoftBody.a


PostBuild.BulletCollision.Release:
/Users/sakya-suzuki/work/kaiserproj/ext/bullet-2.75/mac/src/BulletCollision/Release/libBulletCollision.a:
	/bin/rm -f /Users/sakya-suzuki/work/kaiserproj/ext/bullet-2.75/mac/src/BulletCollision/Release/libBulletCollision.a


PostBuild.BulletDynamics.Release:
/Users/sakya-suzuki/work/kaiserproj/ext/bullet-2.75/mac/src/BulletDynamics/Release/libBulletDynamics.a:
	/bin/rm -f /Users/sakya-suzuki/work/kaiserproj/ext/bullet-2.75/mac/src/BulletDynamics/Release/libBulletDynamics.a


PostBuild.LinearMath.Release:
/Users/sakya-suzuki/work/kaiserproj/ext/bullet-2.75/mac/src/LinearMath/Release/libLinearMath.a:
	/bin/rm -f /Users/sakya-suzuki/work/kaiserproj/ext/bullet-2.75/mac/src/LinearMath/Release/libLinearMath.a


PostBuild.BulletSoftBody.MinSizeRel:
/Users/sakya-suzuki/work/kaiserproj/ext/bullet-2.75/mac/src/BulletSoftBody/MinSizeRel/libBulletSoftBody.a:
	/bin/rm -f /Users/sakya-suzuki/work/kaiserproj/ext/bullet-2.75/mac/src/BulletSoftBody/MinSizeRel/libBulletSoftBody.a


PostBuild.BulletCollision.MinSizeRel:
/Users/sakya-suzuki/work/kaiserproj/ext/bullet-2.75/mac/src/BulletCollision/MinSizeRel/libBulletCollision.a:
	/bin/rm -f /Users/sakya-suzuki/work/kaiserproj/ext/bullet-2.75/mac/src/BulletCollision/MinSizeRel/libBulletCollision.a


PostBuild.BulletDynamics.MinSizeRel:
/Users/sakya-suzuki/work/kaiserproj/ext/bullet-2.75/mac/src/BulletDynamics/MinSizeRel/libBulletDynamics.a:
	/bin/rm -f /Users/sakya-suzuki/work/kaiserproj/ext/bullet-2.75/mac/src/BulletDynamics/MinSizeRel/libBulletDynamics.a


PostBuild.LinearMath.MinSizeRel:
/Users/sakya-suzuki/work/kaiserproj/ext/bullet-2.75/mac/src/LinearMath/MinSizeRel/libLinearMath.a:
	/bin/rm -f /Users/sakya-suzuki/work/kaiserproj/ext/bullet-2.75/mac/src/LinearMath/MinSizeRel/libLinearMath.a


PostBuild.BulletSoftBody.RelWithDebInfo:
/Users/sakya-suzuki/work/kaiserproj/ext/bullet-2.75/mac/src/BulletSoftBody/RelWithDebInfo/libBulletSoftBody.a:
	/bin/rm -f /Users/sakya-suzuki/work/kaiserproj/ext/bullet-2.75/mac/src/BulletSoftBody/RelWithDebInfo/libBulletSoftBody.a


PostBuild.BulletCollision.RelWithDebInfo:
/Users/sakya-suzuki/work/kaiserproj/ext/bullet-2.75/mac/src/BulletCollision/RelWithDebInfo/libBulletCollision.a:
	/bin/rm -f /Users/sakya-suzuki/work/kaiserproj/ext/bullet-2.75/mac/src/BulletCollision/RelWithDebInfo/libBulletCollision.a


PostBuild.BulletDynamics.RelWithDebInfo:
/Users/sakya-suzuki/work/kaiserproj/ext/bullet-2.75/mac/src/BulletDynamics/RelWithDebInfo/libBulletDynamics.a:
	/bin/rm -f /Users/sakya-suzuki/work/kaiserproj/ext/bullet-2.75/mac/src/BulletDynamics/RelWithDebInfo/libBulletDynamics.a


PostBuild.LinearMath.RelWithDebInfo:
/Users/sakya-suzuki/work/kaiserproj/ext/bullet-2.75/mac/src/LinearMath/RelWithDebInfo/libLinearMath.a:
	/bin/rm -f /Users/sakya-suzuki/work/kaiserproj/ext/bullet-2.75/mac/src/LinearMath/RelWithDebInfo/libLinearMath.a




# For each target create a dummy ruleso the target does not have to exist
