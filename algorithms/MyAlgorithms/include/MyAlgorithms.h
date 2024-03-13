#pragma once
#include <string>
#include <vector>
using namespace std;


struct ListNode {
	int val;
	ListNode* next;
	ListNode() : val(0), next(nullptr) {}
	ListNode(int x) : val(x), next(nullptr) {}
	ListNode(int x, ListNode* next) : val(x), next(next) {}
};

struct TreeNode
{
	int val;
	TreeNode* left;
	TreeNode* right;
	TreeNode() :val(0), left(nullptr), right(nullptr) {}
	TreeNode(int x) :val(x), left(nullptr), right(nullptr) {}
	TreeNode(int x, TreeNode* l, TreeNode* r) :val(x), left(l), right(r) {}
};

class MyAlgorithms
{
public:
	///字符串匹配算法
	//\字符串匹配算法:从txt从寻找pat，并返回下标，若没有匹配则返回-1
	//展示普通的搜索算法
	int serachStr(const string& txt, const string& pat);

	//\KMP状态机算法
	//\文档链接:https://zhuanlan.zhihu.com/p/83334559
	int serachStrKMP(const string& txt, const string& pat);
	void createKMPVec(vector<vector<int>>& nums, const string& pat);




	///链表相关
	///反转链表
	ListNode* reverseList(ListNode* head);

	///删除链表的倒数第N个节点
	ListNode* removeNthFromEnd(ListNode* head, int n);




	///二叉树
	///递归遍历
	///三种常见的递归遍历，前中后序遍历
	void traverse(TreeNode* root);

	///https://leetcode.cn/problems/populating-next-right-pointers-in-each-node/description/
	///填充节点的右侧指针
	void connect(TreeNode* node);

	///通过先序遍历和中序遍历构建唯一的二叉树
	TreeNode* BuildTreePreInTree(const vector<int>& preOrder, const vector<int>& inorder);

	TreeNode* createTreePreInTree(const vector<int>& preOrder, const vector<int>& inorder, int pl, int pr, int il, int ir);

	int findElement(int target, const vector<int>& nums);

	///通过后序遍历和中序遍历构建唯一的二叉树
	TreeNode* buildTreePostInTree(const vector<int>& postOrder, const vector<int>& inorder);

	TreeNode* createTreePostInTree(const vector<int>& postOrder, const vector<int>& inorder, int pl, int pr, int il, int ir);
private:
};
