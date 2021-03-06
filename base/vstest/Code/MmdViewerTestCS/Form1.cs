﻿using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using MmdViewerCLR;

namespace MmdViewerTestCS
{
    public partial class Form1 : Form
    {
        private ControlVariableCLRWrapper controlVariableCLRWrapper;
        private Point mousePos;

        public Form1(ControlVariableCLRWrapper controlVariableCLRWrapper)
        {
            this.controlVariableCLRWrapper = controlVariableCLRWrapper;
            InitializeComponent();
            this.MouseWheel += new MouseEventHandler(Form1_mouseWheel);
        }

        // フォーム開始イベント
        // 視点/注視点を設定
        private void Form1_Load(object sender, EventArgs e)
        {
            controlVariableCLRWrapper.eyePoint = new Vector3(10.0f, 20.0f, -50.0f);
            controlVariableCLRWrapper.lookAtPoint = new Vector3(0, 10.0f, 0);
        }

        // メニューOpenボタンクリックイベント
        // ファイルを開く
        private void openOToolStripMenuItem_Click(object sender, EventArgs e)
        {
            var openFileDialog = new OpenFileDialog();
            openFileDialog.Filter = "pmdファイル(*.pmd)|*.pmd|vmdファイル(*.vmd)|*.vmd";
            openFileDialog.RestoreDirectory = true; // これをtrueにしないと、カレントディレクトリがファイルを開いたディレクトリに変更されてしまう！
            if (openFileDialog.ShowDialog() == DialogResult.OK)
            {
                String extension = System.IO.Path.GetExtension(openFileDialog.FileName);
                if (extension == ".pmd")
                {
                    controlVariableCLRWrapper.pmdFileName = openFileDialog.FileName;
                    controlVariableCLRWrapper.openPmdFileFlag = true;
                }
                else if (extension == ".vmd")
                {
                    controlVariableCLRWrapper.vmdFileName = openFileDialog.FileName;
                    controlVariableCLRWrapper.openVmdFileFlag = true;
                }
            }
        }

        // タイマイベント
        // 視点の回転、平行移動
        private void timer1_Tick(object sender, EventArgs e)
        {
            Point mv = new Point(0, 0);
            if (!mousePos.IsEmpty) mv = new Point(Control.MousePosition.X - mousePos.X, Control.MousePosition.Y - mousePos.Y);  // マウスカーソルの移動量
            Vector3 v = controlVariableCLRWrapper.lookAtPoint - controlVariableCLRWrapper.eyePoint; // 視線ベクトル
            if (Control.MouseButtons == MouseButtons.Right)
            {
                controlVariableCLRWrapper.eyePoint = controlVariableCLRWrapper.lookAtPoint - RotateViewVector(v, mv);
                mousePos = Control.MousePosition;
            }
            else if (Control.MouseButtons == MouseButtons.Middle)
            {
                Vector3 t = TranslateViewVector(v, mv);
                controlVariableCLRWrapper.lookAtPoint += t;
                controlVariableCLRWrapper.eyePoint += t;
                mousePos = Control.MousePosition;
            }
            else mousePos = Point.Empty;
        }

        /// <summary>
        /// ベクトルを規格化
        /// </summary>
        /// <param name="v">規格化するベクトル</param>
        /// <returns>規格化したベクトル</returns>
        private Vector3 NormalizeVector3(Vector3 v)
        {
            float r = (float)(Math.Sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z));
            return new Vector3(v.X / r, v.Y / r, v.Z / r);
        }

        /// <summary>
        /// マウスカーソルの移動量に応じて視線ベクトルを回転 
        /// </summary>
        /// <param name="v">視線ベクトル</param>
        /// <param name="mv">カーソルの移動量</param>
        /// <returns>回転した視線ベクトル</returns>
        private Vector3 RotateViewVector(Vector3 v, Point mv)
        {
            const float speed = 0.01f;  // 視線ベクトルの回転スピード
            double r = Math.Sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z);
            double a = Math.Atan2(v.Z, v.X);
            double b = Math.Acos(v.Y / r);
            a -= speed * mv.X;
            b += speed * mv.Y;
            return new Vector3((float)(r * Math.Sin(b) * Math.Cos(a)), (float)(r * Math.Cos(b)), (float)(r * Math.Sin(b) * Math.Sin(a)));
        }

        /// <summary>
        /// カーソルの移動量に応じて視線ベクトルの平行移動量を取得
        /// </summary>
        /// <param name="v">視線ベクトル</param>
        /// <param name="mv">カーソルの移動量</param>
        /// <returns>視線ベクトルの移動量</returns>
        private Vector3 TranslateViewVector(Vector3 v, Point mv)
        {
            const float speed = 0.02f;
            Vector3 x = new Vector3(-v.Z, 0, v.X);  // 視線とY軸に垂直な方向
            x = speed * mv.X * NormalizeVector3(x);
            Vector3 y = new Vector3(-v.Y * v.X, v.Z * v.Z + v.X * v.X, -v.Y * v.Z); // 視線とxに垂直な方向
            y = speed * mv.Y * NormalizeVector3(y);
            return x + y;
        }

        // マウスホイールイベント
        // 視線距離を変更
        private void Form1_mouseWheel(object sender, EventArgs e)
        {
            Vector3 v = controlVariableCLRWrapper.lookAtPoint - controlVariableCLRWrapper.eyePoint; // 視線ベクトル
            v *= 1 - ((MouseEventArgs)e).Delta / 1200.0f;
            controlVariableCLRWrapper.eyePoint = controlVariableCLRWrapper.lookAtPoint - v;
        }

        // メニューPhysics Enabledボタンクリックイベント
        // 物理演算使用可能フラグを切り替える
        private void physicsEnabledToolStripMenuItem_Click(object sender, EventArgs e)
        {
            controlVariableCLRWrapper.physicsEnabled = ((ToolStripMenuItem)sender).Checked;
        }

        // メニューBone Mesh Enabledボタンクリックイベント
        // ボーンメッシュ使用可能フラグを切り替える
        private void boneMeshEnabledToolStripMenuItem_Click(object sender, EventArgs e)
        {
            controlVariableCLRWrapper.boneMeshEnabled = ((ToolStripMenuItem)sender).Checked;
        }

        // メニューRigid Mesh Enabledボタンクリックイベント
        // 剛体メッシュ使用可能フラグを切り替える
        private void rigidMeshEnabledToolStripMenuItem_Click(object sender, EventArgs e)
        {
            controlVariableCLRWrapper.rigidMeshEnabled = ((ToolStripMenuItem)sender).Checked;
        }

        // メニューJoint Mesh Enabledボタンクリックイベント
        // ジョイントメッシュ使用可能フラグを切り替える
        private void jointMeshEnabledToolStripMenuItem_Click(object sender, EventArgs e)
        {
            controlVariableCLRWrapper.jointMeshEnabled = ((ToolStripMenuItem)sender).Checked;
        }



    }
}
